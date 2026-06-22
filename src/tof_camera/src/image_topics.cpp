#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <chronoptics/tof/tui_camera.hpp>
#include <opencv2/opencv.hpp>

namespace tof = chronoptics::tof;

class TOFNode : public rclcpp::Node {
public:
    TOFNode() : Node("tof_camera_node"), cam_("303000c") {
        RCLCPP_INFO(this->get_logger(), "Initializing TOF Camera Node");

        // ROS2 Publishers
        bgr_publisher_ = this->create_publisher<sensor_msgs::msg::Image>("/gordon_tof/bgr", 10);
        intensity_publisher_ = this->create_publisher<sensor_msgs::msg::Image>("/gordon_tof/intensity", 10);
        z_publisher_ = this->create_publisher<sensor_msgs::msg::Image>("/gordon_tof/z", 10);

        // Configuring camera
        cam_.set_output_frame_types({tof::FrameType::BGR, tof::FrameType::INTENSITY, tof::FrameType::Z});
        cam_.start();

        // Drop initial frames
        constexpr int dropFrames = 10;
        for (int i = 0; i < dropFrames; ++i) {
            cam_.get_frames();
        }
        RCLCPP_INFO(this->get_logger(), "Dropped initial frames");

        timer_ = this->create_wall_timer(std::chrono::milliseconds(30), std::bind(&TOFNode::capture_frames, this));
    }

private:
    void capture_frames() {
        // auto start_time = std::chrono::steady_clock::now();
        std::vector<tof::Data> frames = cam_.get_frames();
        // auto after_capture = std::chrono::steady_clock::now();

        for (tof::Data &frame : frames) {
            // auto frame_start_time = std::chrono::steady_clock::now();
            
            if (frame.frame_type() == tof::FrameType::BGR) {
                cv::Mat bgr_img(frame.rows(), frame.cols(), CV_8UC3, frame.data());
                cv::flip(bgr_img, bgr_img, 0);
                publish_image(bgr_img, bgr_publisher_, "bgr_frame", "bgr8");

                // auto frame_end_time = std::chrono::steady_clock::now();
                // double processing_time = std::chrono::duration<double, std::milli>(frame_end_time - frame_start_time).count();
                // RCLCPP_INFO(this->get_logger(), "[TIMING] BGR Processing Time: %.2f ms", processing_time);
            } 
            else if (frame.frame_type() == tof::FrameType::INTENSITY) {
                cv::Mat intensity_img(frame.rows(), frame.cols(), CV_8UC1, frame.data());
                cv::rotate(intensity_img, intensity_img, cv::ROTATE_90_CLOCKWISE);
                cv::flip(intensity_img, intensity_img, 1);
                publish_image(intensity_img, intensity_publisher_, "intensity_frame", "mono8");

                // auto frame_end_time = std::chrono::steady_clock::now();
                // double processing_time = std::chrono::duration<double, std::milli>(frame_end_time - frame_start_time).count();
                // RCLCPP_INFO(this->get_logger(), "[TIMING] Intensity Processing Time: %.2f ms", processing_time);
            } 
            else if (frame.frame_type() == tof::FrameType::Z) {
                cv::Mat z_img(frame.rows(), frame.cols(), CV_32FC1, frame.data());
                double minVal, maxVal;
                cv::minMaxIdx(z_img, &minVal, &maxVal);
                z_img -= minVal;
                z_img /= (maxVal - minVal);
                z_img *= 255.0;
                cv::Mat z_img_scaled;
                z_img.convertTo(z_img_scaled, CV_8UC1);
                cv::rotate(z_img_scaled, z_img_scaled, cv::ROTATE_90_CLOCKWISE);
                cv::flip(z_img_scaled, z_img_scaled, 1);
                publish_image(z_img_scaled, z_publisher_, "z_frame", "mono8");

                // auto frame_end_time = std::chrono::steady_clock::now();
                // double processing_time = std::chrono::duration<double, std::milli>(frame_end_time - frame_start_time).count();
                // RCLCPP_INFO(this->get_logger(), "[TIMING] Z Processing Time: %.2f ms", processing_time);
            }
        }

        // auto end_time = std::chrono::steady_clock::now();
        // double total_time = std::chrono::duration<double, std::milli>(end_time - start_time).count();
        // double capture_time = std::chrono::duration<double, std::milli>(after_capture - start_time).count();
        // double processing_time = std::chrono::duration<double, std::milli>(end_time - after_capture).count();

        // RCLCPP_INFO(this->get_logger(), "[TIMING] Total Frame Capture Time: %.2f ms", capture_time);
        // RCLCPP_INFO(this->get_logger(), "[TIMING] Total Processing Time: %.2f ms", processing_time);
    }

    void publish_image(const cv::Mat &img, rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher, const std::string &frame_id, const std::string &encoding) {
        auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), encoding, img).toImageMsg();
        msg->header.stamp = this->now();
        msg->header.frame_id = frame_id;
        publisher->publish(*msg);
        RCLCPP_INFO(this->get_logger(), "Published %s with encoding %s", frame_id.c_str(), encoding.c_str());
    }

    tof::TuiCamera cam_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr bgr_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr intensity_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr z_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TOFNode>());
    rclcpp::shutdown();
    return 0;
}
