#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <chronoptics/tof/tui_camera.hpp>
#include <chronoptics/tof/calibration.hpp>
#include <chronoptics/tof/data_types.hpp>

namespace tof = chronoptics::tof;

class TOFCalibNode : public rclcpp::Node {
public:
    TOFCalibNode() : Node("tof_camera_calib_node") {
        this->declare_parameter<std::string>("serial_number", "303000c");
        std::string serial_number = this->get_parameter("serial_number").as_string();
        
        try {
            cam_ = tof::TuiCamera(serial_number);
            cam_.set_output_frame_types({tof::FrameType::BGR, tof::FrameType::INTENSITY, tof::FrameType::Z, tof::FrameType::XYZ_BGR});
            cam_.start();
            RCLCPP_INFO(this->get_logger(), "Successfully opened TOF camera with serial %s", serial_number.c_str());
        } catch (...) {
            RCLCPP_WARN(this->get_logger(), "Serial %s not available, trying fallback 303000b", serial_number.c_str());
            try {
                cam_ = tof::TuiCamera("303000b");
                cam_.set_output_frame_types({tof::FrameType::BGR, tof::FrameType::INTENSITY, tof::FrameType::Z, tof::FrameType::XYZ_BGR});
                cam_.start();
            } catch (...) {
                RCLCPP_ERROR(this->get_logger(), "No TOF camera could be opened. Shutting down.");
                rclcpp::shutdown();
                return;
            }
        }

        // Publisher init
        bgr_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/gordon_tof/bgr", rclcpp::SensorDataQoS());
        info_pub_ = this->create_publisher<sensor_msgs::msg::CameraInfo>("/gordon_tof/camera_info", rclcpp::SensorDataQoS());
        intensity_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/gordon_tof/intensity", rclcpp::SensorDataQoS());
        z_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/gordon_tof/z", rclcpp::SensorDataQoS());
        pc_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/gordon_tof/pointcloud", rclcpp::SensorDataQoS());

        // Drop initial frames
        // for (int i = 0; i < 10; ++i) cam_.get_frames();
  
        try {
            for (int i = 0; i < 10; ++i) cam_.get_frames();
        } catch (const std::exception &e) {
            RCLCPP_WARN(this->get_logger(), "Exception while dropping initial frames: %s", e.what());
        }

        timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&TOFCalibNode::capture_frames, this)); 
    }

private:
    void capture_frames() {
        // auto frames = cam_.get_frames();
      
        std::vector<tof::Data> frames;
        try {
            frames = cam_.get_frames();
        } catch (const std::exception &e) {
            RCLCPP_WARN(this->get_logger(), "Exception while capturing frames: %s", e.what());
            return;
        }

        auto stamp = this->now();

        for (tof::Data &frame : frames) {
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
            }
        }        
    }

    void publish_camera_info(const rclcpp::Time &stamp, int rows, int cols) {
        sensor_msgs::msg::CameraInfo info;
        info.header.stamp = stamp;
        info.header.frame_id = "tof_optical_frame";
        
        auto calib = cam_.get_calibration();

        auto rgb_mat = calib.get_rgb_camera_matrix();
        auto rgb_dist = calib.get_rgb_distortion_coefficients();

        info.height = rows; 
        info.width = cols; 

        double fx = rgb_mat[0];
        double fy= rgb_mat[4];
        double cx = rgb_mat[2];
        double cy = rgb_mat[5];

        info.k = {fx, 0.0, cx,
                  0.0, fy, cy,
                  0.0, 0.0, 1.0};

        info.p = {fx, 0.0, cx, 0.0,
                  0.0, fy, cy, 0.0,
                  0.0, 0.0, 1.0, 0.0};

        info.d = {rgb_dist[0], rgb_dist[1], rgb_dist[2], rgb_dist[3], rgb_dist[4]};

        info.distortion_model = "plumb_bob";
        info.r = {1.0, 0.0, 0.0,
                  0.0, 1.0, 0.0,
                  0.0, 0.0, 1.0};

        info_pub_->publish(info);
    }

    void handle_bgr(tof::Data &frame, const rclcpp::Time &stamp) {
        cv::Mat img(frame.rows(), frame.cols(), CV_8UC3, frame.data());
        // 🔹 Rotazione 90 gradi Clockwise
        // cv::Mat rotated;
        // cv::rotate(img, rotated, cv::ROTATE_90_CLOCKWISE);
        
        publish_image(img, bgr_pub_, "tof_optical_frame", "bgr8", stamp);
        publish_camera_info(stamp, frame.rows(), frame.cols());

        // // 🔹 Visualizzazione locale
        // cv::imshow("TOF BGR Visual", img);
        // cv::waitKey(1);
    }

    void handle_intensity(tof::Data &frame, const rclcpp::Time &stamp) {
        cv::Mat img(frame.rows(), frame.cols(), CV_8UC1, frame.data());
        // 🔹 Intensity: Transpose (matching node: 90 CW + Flip H)
        cv::Mat rotated;
        cv::rotate(img, rotated, cv::ROTATE_90_CLOCKWISE);
        cv::flip(rotated, rotated, -1);
        publish_image(rotated, intensity_pub_, "tof_optical_frame", "mono8", stamp);
    }

    void handle_z(tof::Data &frame, const rclcpp::Time &stamp) {
        cv::Mat img(frame.rows(), frame.cols(), CV_32FC1, frame.data());
        double minVal, maxVal;
        cv::minMaxIdx(img, &minVal, &maxVal);
        img = (img - minVal) / std::max(1e-6, (maxVal - minVal)) * 255.0;
        cv::Mat scaled;
        img.convertTo(scaled, CV_8UC1);
        // 🔹 Z: Transpose (matching node: 90 CW + Flip H)
        cv::Mat rotated;
        cv::rotate(scaled, rotated, cv::ROTATE_90_CLOCKWISE);
        cv::flip(rotated, rotated, -1);
        publish_image(rotated, z_pub_, "tof_optical_frame", "mono8", stamp);
    }

    void handle_pointcloud(tof::Data &frame, const rclcpp::Time &stamp, bool with_bgr=false) {
        sensor_msgs::msg::PointCloud2 msg;
        msg.header.stamp = stamp;
        msg.header.frame_id = "tof_link";
        msg.height = frame.rows();
        msg.width = frame.cols();
        msg.is_dense = false;
        msg.is_bigendian = false;

        sensor_msgs::PointCloud2Modifier modifier(msg);
        if(with_bgr) {
            modifier.setPointCloud2Fields(4, "x", 1, sensor_msgs::msg::PointField::FLOAT32,
                                             "y", 1, sensor_msgs::msg::PointField::FLOAT32,
                                             "z", 1, sensor_msgs::msg::PointField::FLOAT32,
                                             "rgb", 1, sensor_msgs::msg::PointField::UINT32);
            modifier.resize(frame.rows() * frame.cols());
            tof::XYZBGR* data_ptr = reinterpret_cast<tof::XYZBGR*>(frame.data());
            sensor_msgs::PointCloud2Iterator<float> iter_x(msg, "x"), iter_y(msg, "y"), iter_z(msg, "z");
            sensor_msgs::PointCloud2Iterator<unsigned int> iter_rgb(msg, "rgb");
            
            for (size_t i = 0; i < frame.rows() * frame.cols(); ++i) {
                *iter_x = data_ptr[i].x * 0.001f;
                *iter_y = data_ptr[i].y * 0.001f;
                *iter_z = data_ptr[i].z * 0.001f;
                
                *iter_rgb = data_ptr[i].r << 16 | data_ptr[i].g << 8 | data_ptr[i].b ;
                ++iter_x; ++iter_y; ++iter_z; ++iter_rgb;
            }
            
        }
        else
        {
            modifier.setPointCloud2Fields(3, "x", 1, sensor_msgs::msg::PointField::FLOAT32,
                                            "y", 1, sensor_msgs::msg::PointField::FLOAT32,
                                            "z", 1, sensor_msgs::msg::PointField::FLOAT32);
            modifier.resize(frame.rows() * frame.cols());

            float* data_ptr = reinterpret_cast<float*>(frame.data());
            sensor_msgs::PointCloud2Iterator<float> iter_x(msg, "x"), iter_y(msg, "y"), iter_z(msg, "z");

            for (size_t i = 0; i < frame.rows() * frame.cols(); ++i) {
            *iter_x = data_ptr[i * 4 + 0] * 0.001f;
            *iter_y = data_ptr[i * 4 + 1] * 0.001f;
            *iter_z = data_ptr[i * 4 + 2] * 0.001f;
            ++iter_x; ++iter_y; ++iter_z;
            }
        }
        pc_pub_->publish(msg);
    }

    void publish_image(cv::Mat &img, rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub, const std::string &frame_id, const std::string &encoding, const rclcpp::Time &stamp) {
        std_msgs::msg::Header header;
        header.stamp = stamp;
        header.frame_id = frame_id;
        sensor_msgs::msg::Image::SharedPtr msg = cv_bridge::CvImage(header, encoding, img).toImageMsg();
        pub->publish(*msg);
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr bgr_pub_, intensity_pub_, z_pub_;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr info_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pc_pub_;
    tof::TuiCamera cam_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TOFCalibNode>());
    rclcpp::shutdown();
    return 0;
}