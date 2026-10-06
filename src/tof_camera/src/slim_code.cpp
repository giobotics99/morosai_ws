#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>
#include <chronoptics/tof/tui_camera.hpp>
#include <chronoptics/tof/data_types.hpp>

namespace tof = chronoptics::tof;

class TOFNode : public rclcpp::Node {
public:
    TOFNode() : Node("tof_camera_node"), cam_("303000c") {
        RCLCPP_INFO(this->get_logger(), "Starting TOF camera node...");

        bgr_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/gordon_tof/bgr", 10);
        intensity_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/gordon_tof/intensity", 10);
        z_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/gordon_tof/z", 10);
        pc_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/gordon_tof/pointcloud", 10);

        cam_.set_output_frame_types({tof::FrameType::BGR, tof::FrameType::INTENSITY, tof::FrameType::Z, tof::FrameType::XYZ_BGR});
        cam_.start();

        // Drop initial frames
        for (int i = 0; i < 10; ++i) cam_.get_frames();

        timer_ = this->create_wall_timer(std::chrono::milliseconds(30), std::bind(&TOFNode::capture_frames, this));
    }

private:
    void capture_frames() {
        auto frames = cam_.get_frames();

        for (tof::Data &frame : frames) {
            switch (frame.frame_type()) {
                case tof::FrameType::BGR:
                    handle_bgr(frame);
                    break;
                case tof::FrameType::INTENSITY:
                    handle_intensity(frame);
                    break;
                case tof::FrameType::Z:
                    handle_z(frame);
                    break;
                case tof::FrameType::XYZ_BGR:
                    handle_pointcloud(frame);
                    break;
                default:
                    break;
            }
        }
    }

    void handle_bgr(tof::Data &frame) {
        cv::Mat img(frame.rows(), frame.cols(), CV_8UC3, frame.data());
        cv::flip(img, img, 0);
        publish_image(img, bgr_pub_, "bgr_frame", "bgr8");
    }

    void handle_intensity(tof::Data &frame) {
        cv::Mat img(frame.rows(), frame.cols(), CV_8UC1, frame.data());
        cv::rotate(img, img, cv::ROTATE_90_CLOCKWISE);
        cv::flip(img, img, 1);
        publish_image(img, intensity_pub_, "intensity_frame", "mono8");
    }

    void handle_z(tof::Data &frame) {
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

    void handle_pointcloud(tof::Data &frame) {
        sensor_msgs::msg::PointCloud2 cloud;
        cloud.header.stamp = this->now();
        cloud.header.frame_id = "camera_link";
        cloud.height = frame.rows();
        cloud.width = frame.cols();
        cloud.is_dense = false;
        cloud.is_bigendian = false;

        sensor_msgs::PointCloud2Modifier modifier(cloud);
        modifier.setPointCloud2Fields(4, "x", 1, sensor_msgs::msg::PointField::FLOAT32,
                                         "y", 1, sensor_msgs::msg::PointField::FLOAT32,
                                         "z", 1, sensor_msgs::msg::PointField::FLOAT32,
                                         "rgb", 1, sensor_msgs::msg::PointField::UINT32);
        modifier.resize(frame.rows() * frame.cols());

        tof::XYZBGR* data_ptr = reinterpret_cast<tof::XYZBGR*>(frame.data());
        sensor_msgs::PointCloud2Iterator<float> iter_x(cloud, "x"), iter_y(cloud, "y"), iter_z(cloud, "z");
        sensor_msgs::PointCloud2Iterator<uint32_t> iter_rgb(cloud, "rgb");

        for (size_t i = 0; i < frame.rows() * frame.cols(); ++i) {
            *iter_x = data_ptr[i].x * 0.001f;
            *iter_y = data_ptr[i].y * 0.001f;
            *iter_z = data_ptr[i].z * 0.001f;

            *iter_rgb = (data_ptr[i].r << 16) | (data_ptr[i].g << 8) | (data_ptr[i].b);

            ++iter_x; ++iter_y; ++iter_z; ++iter_rgb;
        }

        pc_pub_->publish(cloud);
    }

    void publish_image(cv::Mat &img, rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub, const std::string &frame_id, const std::string &encoding) {
        std_msgs::msg::Header header;
        header.stamp = this->now();
        header.frame_id = frame_id;
        sensor_msgs::msg::Image::SharedPtr msg = cv_bridge::CvImage(header, encoding, img).toImageMsg();
        pub->publish(*msg);
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr bgr_pub_, intensity_pub_, z_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pc_pub_;
    tof::TuiCamera cam_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TOFNode>());
    rclcpp::shutdown();
    return 0;
}
