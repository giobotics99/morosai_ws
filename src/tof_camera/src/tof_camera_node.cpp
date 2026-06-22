#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <chronoptics/tof/tui_camera.hpp>
#include <chronoptics/tof/data_types.hpp>

namespace tof = chronoptics::tof;

class TOFNode : public rclcpp::Node {
public:
    TOFNode() : Node("tof_camera_node") {
        std::string serial_number = "303000c";  // quello che vuoi provare
        bool cam_started = false;

        try {
            cam_ = tof::TuiCamera(serial_number);
            cam_.set_output_frame_types({tof::FrameType::BGR, tof::FrameType::INTENSITY, tof::FrameType::Z, tof::FrameType::XYZ_BGR});
            cam_.start();
            cam_started = true;
        } catch (...) {
            RCLCPP_WARN(this->get_logger(), "Serial %s not available, trying fallback 303000b", serial_number.c_str());
            try {
                cam_ = tof::TuiCamera("303000b");
                cam_.set_output_frame_types({tof::FrameType::BGR, tof::FrameType::INTENSITY, tof::FrameType::Z, tof::FrameType::XYZ_BGR});
                cam_.start();
                cam_started = true;
            } catch (...) {
                RCLCPP_ERROR(this->get_logger(), "No TOF camera could be opened. Shutting down.");
                rclcpp::shutdown();
                return;
            }
        }

        // Publisher init
        bgr_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/gordon_tof/bgr", 10);
        intensity_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/gordon_tof/intensity", 10);
        z_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/gordon_tof/z", 10);
        pc_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/gordon_tof/pointcloud", 10);

        // Camera config: output all required frame types
        cam_.set_output_frame_types({tof::FrameType::BGR, tof::FrameType::INTENSITY, tof::FrameType::Z, tof::FrameType::XYZ_BGR});
        cam_.start();
        auto names = cam_.get_multiple_names();
        auto descriptions = cam_.get_multiple_descriptions();

        for (size_t i = 0; i < names.size(); i++) {
            std::cout << i << " " << names[i] << "\n\t" << descriptions[i] << "\n";
        }
        std::string log_string = "Opened camera with Serial Number: " + std::string(cam_.get_serial()) + "\n Config index: " + std::to_string(cam_.config_index());
        RCLCPP_INFO_ONCE(this->get_logger(), log_string.c_str());
        
        // Drop first N frames
        for (int i = 0; i < 10; ++i) cam_.get_frames();

        // timer_ = this->create_wall_timer(std::chrono::milliseconds(30), std::bind(&TOFNode::capture_frames, this)); // 30 frames per second
        timer_ = this->create_wall_timer(std::chrono::milliseconds(200), std::bind(&TOFNode::capture_frames, this)); // 5 frames per second (T = 1/5hz = 200ms)
    }

private:
    tof::Data xyz_frame_, bgr_frame_;
    bool has_xyz_ = false, has_bgr_ = false;

    void capture_frames() {
        auto frames = cam_.get_frames();
        has_xyz_ = has_bgr_ = false;

        for (tof::Data &frame : frames) {
            switch (frame.frame_type()) {
                case tof::FrameType::BGR:
                    bgr_frame_ = std::move(frame);
                    has_bgr_ = true;
                    handle_bgr(bgr_frame_);
                    break;
                case tof::FrameType::INTENSITY:
                    handle_intensity(frame);
                    break;
                case tof::FrameType::Z:
                    handle_z(frame);
                    break;
                case tof::FrameType::XYZ:
                    xyz_frame_ = std::move(frame);
                    has_xyz_ = true;
                    handle_pointcloud(xyz_frame_);
                    break;
                case tof::FrameType::XYZ_BGR:
                    has_xyz_ = true;
                    handle_pointcloud(frame, true);
                    // publish_bgr_and_z_from_xyzbgr(frame);
                    break;
            }
        }        

    }

    void handle_bgr(tof::Data &frame) {
        std::string log_string = "Handling BGR frame - Rows: " + std::to_string(frame.rows()) + " Cols: " + std::to_string(frame.cols());
        RCLCPP_INFO_ONCE(this->get_logger(), log_string.c_str());
        cv::Mat img(frame.rows(), frame.cols(), CV_8UC3, frame.data());
        cv::flip(img, img, -1);
        publish_image(img, bgr_pub_, "bgr_frame", "bgr8");
    }

    void handle_intensity(tof::Data &frame) {
        std::string log_string = "Handling Intensity frame - Rows: " + std::to_string(frame.rows()) + " Cols: " + std::to_string(frame.cols());
        RCLCPP_INFO_ONCE(this->get_logger(), log_string.c_str());
        cv::Mat img(frame.rows(), frame.cols(), CV_8UC1, frame.data());
        cv::rotate(img, img, cv::ROTATE_90_CLOCKWISE);
        cv::flip(img, img, 1);
        publish_image(img, intensity_pub_, "intensity_frame", "mono8");
    }

    void handle_z(tof::Data &frame) {
        std::string log_string = "Handling Z frame - Rows: " + std::to_string(frame.rows()) + " Cols: " + std::to_string(frame.cols());
        RCLCPP_INFO_ONCE(this->get_logger(), log_string.c_str());
        cv::Mat img(frame.rows(), frame.cols(), CV_32FC1, frame.data());
        double minVal, maxVal;
        cv::minMaxIdx(img, &minVal, &maxVal);
        img = (img - minVal) / (maxVal - minVal) * 255.0;
        cv::Mat scaled;
        img.convertTo(scaled, CV_8UC1);
        cv::rotate(scaled, scaled, cv::ROTATE_90_CLOCKWISE);
        cv::flip(scaled, scaled, 1);
        publish_image(scaled, z_pub_, "z_frame", "mono8");
    }

    void handle_pointcloud(tof::Data &frame, bool with_bgr=false) {
        std::string log_string = "Handling XYZ frame - Rows: " + std::to_string(frame.rows()) + " Cols: " + std::to_string(frame.cols());
        RCLCPP_INFO_ONCE(this->get_logger(), log_string.c_str());
        sensor_msgs::msg::PointCloud2 msg;
        msg.header.stamp = this->now();
        msg.header.frame_id = "tof_link";
        msg.height = frame.rows();
        msg.width = frame.cols();
        msg.is_dense = false;
        msg.is_bigendian = false;

        sensor_msgs::PointCloud2Modifier modifier(msg);
        if(with_bgr)
        {
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


    void publish_image(cv::Mat &img, rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub, const std::string &frame_id, const std::string &encoding) {
        std_msgs::msg::Header header;
        header.stamp = this->now();
        header.frame_id = frame_id;
        sensor_msgs::msg::Image::SharedPtr msg = cv_bridge::CvImage(header, encoding, img).toImageMsg();
        pub->publish(*msg);
    }

    // // Pubblica /gordon_tof/bgr e /gordon_tof/z a partire da un frame XYZ_BGR
    // void publish_bgr_and_z_from_xyzbgr(tof::Data &frame) {
    //     cv::Mat bgr(frame.rows(), frame.cols(), CV_8UC3);
    //     cv::Mat z_float(frame.rows(), frame.cols(), CV_32FC1);

    //     auto* data_ptr = reinterpret_cast<tof::XYZBGR*>(frame.data());

    //     for (int r = 0; r < frame.rows(); ++r) {
    //         for (int c = 0; c < frame.cols(); ++c) {
    //             size_t i = static_cast<size_t>(r) * frame.cols() + c;

    //             // BGR (OpenCV usa BGR)
    //             cv::Vec3b &pix = bgr.at<cv::Vec3b>(r, c);
    //             pix[0] = data_ptr[i].b;
    //             pix[1] = data_ptr[i].g;
    //             pix[2] = data_ptr[i].r;

    //             // Z in metri
    //             z_float.at<float>(r, c) = data_ptr[i].z * 0.001f;
    //         }
    //     }

    //     // BGR: stesso orientamento di handle_bgr
    //     cv::flip(bgr, bgr, -1);
    //     publish_image(bgr, bgr_pub_, "bgr_frame", "bgr8");

    //     // Z: normalizza come handle_z (rotate 90 CW, poi flip)
    //     double minVal, maxVal;
    //     cv::minMaxIdx(z_float, &minVal, &maxVal);
    //     cv::Mat z_norm = (z_float - minVal) / std::max(1e-6, (maxVal - minVal)) * 255.0;
    //     cv::Mat z_mono8;
    //     z_norm.convertTo(z_mono8, CV_8UC1);
    //     cv::rotate(z_mono8, z_mono8, cv::ROTATE_90_CLOCKWISE);
    //     cv::flip(z_mono8, z_mono8, 1);
    //     publish_image(z_mono8, z_pub_, "z_frame", "mono8");
    // }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr bgr_pub_, intensity_pub_, z_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pc_pub_, pc_pub_colored_;
    tof::TuiCamera cam_;
};


int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TOFNode>());
    rclcpp::shutdown();
    return 0;
}
