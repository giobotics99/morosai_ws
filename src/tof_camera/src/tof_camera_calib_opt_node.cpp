#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <pcl_conversions/pcl_conversions.h>

#include <chronoptics/tof/tui_camera.hpp>
#include <chronoptics/tof/calibration.hpp>
#include <chronoptics/tof/data_types.hpp>

namespace tof = chronoptics::tof;

// ─────────────────────────────────────────────────────────────────────────────
// TOFCalibNode – ingel (direct/upright) camera mount configuration.
//
// Raw sensor BGR always comes out rotated 180° (sensor-level quirk, independent
// of physical mount orientation).  We therefore ALWAYS apply:
//   • BGR:       cv::flip(img, img, -1)   → 180° correction
//   • camera_K:  cx' = W-1-cx,  cy' = H-1-cy  → mirrors principal point to
//                match the flipped image
//   • Intensity/Z: rotate90CW then flipH  → sensor-level axis correction
//   • PointCloud: iterate forward (no reversal), XYZ from sensor are already
//                 in the camera optical frame (Z forward, X right, Y down).
// ─────────────────────────────────────────────────────────────────────────────

class TOFCalibNode : public rclcpp::Node {
public:
    TOFCalibNode() : Node("tof_camera_calib_opt_node") {
        this->declare_parameter<std::string>("serial_number", "303000c");
        serial_number_ = this->get_parameter("serial_number").as_string();

        // Publishers
        bgr_pub_       = this->create_publisher<sensor_msgs::msg::Image>("/gordon_tof/bgr", rclcpp::SensorDataQoS());
        info_pub_      = this->create_publisher<sensor_msgs::msg::CameraInfo>("/gordon_tof/camera_info", rclcpp::SensorDataQoS());
        intensity_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/gordon_tof/intensity", rclcpp::SensorDataQoS());
        z_pub_         = this->create_publisher<sensor_msgs::msg::Image>("/gordon_tof/z", rclcpp::SensorDataQoS());
        pc_pub_        = this->create_publisher<sensor_msgs::msg::PointCloud2>("/gordon_tof/pointcloud", rclcpp::SensorDataQoS());

        if (!initialize_camera(serial_number_)) {
            RCLCPP_WARN(this->get_logger(), "Serial %s not available, trying fallback 303000b", serial_number_.c_str());
            if (!initialize_camera("303000b")) {
                RCLCPP_ERROR(this->get_logger(), "No camera found at startup.");
            } else {
                serial_number_ = "303000b";
            }
        }

        // Capture timer (10 Hz)
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&TOFCalibNode::capture_frames, this)
        );
    }

    bool has_crashed() const { return has_crashed_; }

private:
    bool initialize_camera(const std::string& serial) {
        try {
            cam_ = tof::TuiCamera(serial);
            cam_.switch_config(1);  // 1 = default config with all frames enabled
            cam_.set_output_frame_types({
                tof::FrameType::BGR,
                tof::FrameType::INTENSITY,
                tof::FrameType::Z,
                tof::FrameType::XYZ_BGR
            });
            cam_.start();

            // Discard first frames to let auto-exposure stabilise
            for (int i = 0; i < 5; ++i) cam_.get_frames();

            camera_initialized_ = true;
            RCLCPP_INFO(this->get_logger(), "Camera %s initialised successfully.", serial.c_str());
            return true;
        } catch (const std::exception& e) {
            camera_initialized_ = false;
            return false;
        }
    }

    void capture_frames() {
        if (!camera_initialized_) {
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Attempting reconnection...");
            initialize_camera(serial_number_);
            return;
        }

        try {
            auto frames = cam_.get_frames();
            auto stamp  = this->now();

            for (auto& frame : frames) {
                switch (frame.frame_type()) {
                    case tof::FrameType::BGR:
                        handle_bgr(frame, stamp);
                        break;
                    case tof::FrameType::INTENSITY:
                        handle_intensity(frame, stamp);
                        break;
                    case tof::FrameType::Z:
                        handle_z(frame, stamp);
                        break;
                    case tof::FrameType::XYZ_BGR:
                        handle_pointcloud(frame, stamp, true);
                        break;
                    default:
                        break;
                }
            }
        } catch (const std::exception& e) {
            RCLCPP_WARN(this->get_logger(), "Capture error: %s. Resetting stream...", e.what());
            camera_initialized_ = false;
        }
    }

    // ── BGR ──────────────────────────────────────────────────────────────────
    // The raw sensor BGR frame is always 180° rotated (sensor quirk).
    // Apply flip(-1) unconditionally so the published image is upright.
    void handle_bgr(tof::Data& frame, const rclcpp::Time& stamp) {
        cv::Mat img(frame.rows(), frame.cols(), CV_8UC3, frame.data());
        cv::flip(img, img, -1);  // sensor-level 180° correction (always needed)
        publish_image(img, bgr_pub_, "tof_optical_frame", "bgr8", stamp);
        publish_camera_info(stamp, frame.rows(), frame.cols());
    }

    // ── Intensity ─────────────────────────────────────────────────────────────
    // Rotate 90° CW then flip horizontally – sensor-level axis correction.
    void handle_intensity(tof::Data& frame, const rclcpp::Time& stamp) {
        cv::Mat img(frame.rows(), frame.cols(), CV_8UC1, frame.data());
        cv::Mat rotated;
        cv::rotate(img, rotated, cv::ROTATE_90_CLOCKWISE);
        cv::flip(rotated, rotated, 1);  // sensor-level horizontal mirror
        publish_image(rotated, intensity_pub_, "tof_optical_frame", "mono8", stamp);
    }

    // ── Z depth ───────────────────────────────────────────────────────────────
    // Same axis correction as intensity.
    void handle_z(tof::Data& frame, const rclcpp::Time& stamp) {
        cv::Mat img(frame.rows(), frame.cols(), CV_32FC1, frame.data());

        double minVal, maxVal;
        cv::minMaxIdx(img, &minVal, &maxVal);
        img = (img - minVal) / std::max(1e-6, (maxVal - minVal)) * 255.0;

        cv::Mat scaled, rotated;
        img.convertTo(scaled, CV_8UC1);
        cv::rotate(scaled, rotated, cv::ROTATE_90_CLOCKWISE);
        cv::flip(rotated, rotated, 1);  // sensor-level horizontal mirror

        publish_image(rotated, z_pub_, "tof_optical_frame", "mono8", stamp);
    }

    // ── PointCloud ────────────────────────────────────────────────────────────
    // XYZ values from the sensor are expressed in the camera optical frame
    // (Z forward, X right, Y down).  For the ingel (direct) mount the sensor
    // data is already correct – iterate forward without reversal.
    void handle_pointcloud(tof::Data& frame, const rclcpp::Time& stamp, bool with_bgr) {
        sensor_msgs::msg::PointCloud2 msg;
        msg.header.stamp    = stamp;
        msg.header.frame_id = "tof_link";
        msg.height          = frame.rows();
        msg.width           = frame.cols();
        msg.is_dense        = false;

        sensor_msgs::PointCloud2Modifier modifier(msg);
        const size_t N = frame.rows() * frame.cols();

        if (with_bgr) {
            modifier.setPointCloud2FieldsByString(2, "xyz", "rgb");
            modifier.resize(N);

            auto* data_ptr = reinterpret_cast<tof::XYZBGR*>(frame.data());
            sensor_msgs::PointCloud2Iterator<float>   iter_x(msg, "x"), iter_y(msg, "y"), iter_z(msg, "z");
            sensor_msgs::PointCloud2Iterator<uint8_t> iter_rgb(msg, "rgb");

            for (size_t i = 0; i < N; ++i) {
                *iter_x    = data_ptr[i].x * 0.001f;
                *iter_y    = data_ptr[i].y * 0.001f;
                *iter_z    = data_ptr[i].z * 0.001f;
                iter_rgb[0] = data_ptr[i].b;
                iter_rgb[1] = data_ptr[i].g;
                iter_rgb[2] = data_ptr[i].r;
                ++iter_x; ++iter_y; ++iter_z; ++iter_rgb;
            }
        }
        pc_pub_->publish(msg);
    }

    // ── Camera info ───────────────────────────────────────────────────────────
    // Because the published BGR image has been flipped 180° relative to the
    // raw sensor frame, the principal point (cx, cy) must be mirrored to
    // remain consistent:
    //
    //   cx' = (W - 1) - cx_sensor
    //   cy' = (H - 1) - cy_sensor
    //
    // fx and fy are invariant under a 180° flip.
    // rows/cols are those of the RAW frame (BGR is not resized, only flipped).
    void publish_camera_info(const rclcpp::Time& stamp, int rows, int cols) {
        sensor_msgs::msg::CameraInfo info;
        info.header.stamp    = stamp;
        info.header.frame_id = "tof_optical_frame";
        info.height          = rows;
        info.width           = cols;

        auto calib    = cam_.get_calibration();
        auto k_mat    = calib.get_rgb_camera_matrix();   // [fx, 0, cx, 0, fy, cy, 0, 0, 1]
        auto d_coeffs = calib.get_rgb_distortion_coefficients();

        const double fx          = k_mat[0];
        const double fy          = k_mat[4];
        const double cx_sensor   = k_mat[2]; 
        const double cy_sensor   = k_mat[5];

        // Mirror principal point to match the 180° flipped image
        const double cx = static_cast<double>(cols) - 1.0 - cx_sensor;
        const double cy = static_cast<double>(rows) - 1.0 - cy_sensor;

        info.k = {fx,  0.0, cx,
                  0.0, fy,  cy,
                  0.0, 0.0, 1.0};

        info.p = {-fx,  0.0, cx,  0.0,
                  0.0, -fy,  cy,  0.0,
                  0.0, 0.0, 1.0, 0.0};

        info.r = {1.0, 0.0, 0.0,
                  0.0, 1.0, 0.0,
                  0.0, 0.0, 1.0};

        info.d = {d_coeffs[0], d_coeffs[1], d_coeffs[2], d_coeffs[3], d_coeffs[4]};
        info.distortion_model = "plumb_bob";

        info_pub_->publish(info);
    }

    void publish_image(cv::Mat& img,
                       rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub,
                       const std::string& frame_id,
                       const std::string& encoding,
                       const rclcpp::Time& stamp)
    {
        std_msgs::msg::Header header;
        header.stamp    = stamp;
        header.frame_id = frame_id;
        pub->publish(*cv_bridge::CvImage(header, encoding, img).toImageMsg());
    }

    // ── Members ───────────────────────────────────────────────────────────────
    tof::TuiCamera cam_;
    std::string serial_number_;
    bool camera_initialized_ = false;
    bool has_crashed_        = false;

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr     bgr_pub_, intensity_pub_, z_pub_;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr info_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pc_pub_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TOFCalibNode>();

    try {
        rclcpp::spin(node);
    } catch (const std::exception& e) {
        RCLCPP_FATAL(rclcpp::get_logger("tof_node"), "Fatal error: %s", e.what());
    }

    node.reset();
    rclcpp::shutdown();
    return 0;
}