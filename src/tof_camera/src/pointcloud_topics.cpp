#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <chronoptics/tof/tui_camera.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

namespace tof = chronoptics::tof;

// Classe principale che rappresenta il nodo ROS per la fotocamera TOF (Time-of-Flight)
class TOFNode : public rclcpp::Node {
public:
    TOFNode() : Node("tof_camera_node"), cam_("303000c") {
        RCLCPP_INFO(this->get_logger(), "Initializing TOF Camera Node");

        pointcloud_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/gordon_tof/pointcloud", 10);

        cam_.set_output_frame_types({tof::FrameType::XYZ});
        cam_.start();

        timer_ = this->create_wall_timer(std::chrono::milliseconds(30), std::bind(&TOFNode::capture_frames, this));
    }

private:
    // Funzione per catturare i frame dalla fotocamera TOF
    void capture_frames() {
        auto frames = cam_.get_frames();
        for (tof::Data &frame : frames) {
            if (frame.frame_type() == tof::FrameType::XYZ) {
                RCLCPP_INFO(this->get_logger(), "Captured frame with %d rows and %d cols", frame.rows(), frame.cols());
    
                // Debug: print the first few points
                float* data_ptr = reinterpret_cast<float*>(frame.data());
                //for (size_t i = 0; i < std::min<size_t>(10, frame.rows() * frame.cols()); ++i) {
                for (size_t i = 0; i < frame.cols(); ++i) {
                    int ii = frame.rows()*0.5*frame.cols()*4;
                    RCLCPP_INFO(this->get_logger(), "Point %zu: x=%f, y=%f, z=%f", i,
                        data_ptr[ii + i * 4], data_ptr[ii + i * 4 + 1], data_ptr[ii + i * 4 + 2]);
                }
    
                publish_pointcloud(frame);
            } else {
                RCLCPP_WARN(this->get_logger(), "Non-XYZ frame type received");
            }
        }
    }

    // Funzione per creare e pubblicare il messaggio PointCloud2
    void publish_pointcloud(tof::Data &frame) {
        RCLCPP_INFO(this->get_logger(), "Publishing PointCloud2 message");
    
        // Debug: print raw data
        float* data_ptr = reinterpret_cast<float*>(frame.data());
        // for (size_t i = 0; i < std::min<size_t>(10, frame.rows() * frame.cols()); ++i) {
        //     RCLCPP_INFO(this->get_logger(), "Raw data: x=%f, y=%f, z=%f",
        //         data_ptr[i * 4], data_ptr[i * 4 + 1], data_ptr[i * 4 + 2]);
        // }
    
        // Create PointCloud2 message
        sensor_msgs::msg::PointCloud2 pointcloud_msg;
        pointcloud_msg.header.stamp = this->now();
        pointcloud_msg.header.frame_id = "camera_link";
        pointcloud_msg.height = frame.rows();
        pointcloud_msg.width = frame.cols();
        pointcloud_msg.is_dense = false;
        pointcloud_msg.is_bigendian = false;
    
        // Set PointCloud2 fields
        sensor_msgs::PointCloud2Modifier modifier(pointcloud_msg);
        modifier.setPointCloud2Fields(3, 
            "x", 1, sensor_msgs::msg::PointField::FLOAT32,
            "y", 1, sensor_msgs::msg::PointField::FLOAT32,
            "z", 1, sensor_msgs::msg::PointField::FLOAT32);
        modifier.resize(frame.rows() * frame.cols());
    
        // Populate PointCloud2 message
        sensor_msgs::PointCloud2Iterator<float> iter_x(pointcloud_msg, "x");
        sensor_msgs::PointCloud2Iterator<float> iter_y(pointcloud_msg, "y");
        sensor_msgs::PointCloud2Iterator<float> iter_z(pointcloud_msg, "z");
    
        for (size_t i = 0; i < frame.rows() * frame.cols(); ++i) {
            *iter_x = data_ptr[i * 4]* 0.001f; // *0.01f to convert from cm to m NON SO UNITA' DI MISURA!!!!!
            *iter_y = data_ptr[i * 4 + 1]* 0.001f;
            *iter_z = data_ptr[i * 4 + 2]* 0.001f;
            ++iter_x; ++iter_y; ++iter_z;
        }
    
        // Debug: confirm the number of points published
        RCLCPP_INFO(this->get_logger(), "Publishing %d points to the pointcloud", frame.rows() * frame.cols());
    
        // Publish the message
        pointcloud_publisher_->publish(pointcloud_msg);
    }
    
    tof::TuiCamera cam_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_publisher_; // Publisher per inviare il messaggio PointCloud2
    rclcpp::TimerBase::SharedPtr timer_; // Timer per catturare i frame a intervalli regolari
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TOFNode>());
    rclcpp::shutdown();
    return 0;
}

// distanze su rviz da checkare
