#include <chrono>
#include <memory>
#include <string>

#include <librealsense2/rs.hpp>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/vector3_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

class T265BridgeNode : public rclcpp::Node {
public:
    T265BridgeNode() : Node("realsense_t265_node") {
        // Inizializzazione Publisher con messaggi standard ROS2
        gyro_pub_ = this->create_publisher<geometry_msgs::msg::Vector3Stamped>("/gyro/sample", 100);
        accel_pub_ = this->create_publisher<geometry_msgs::msg::Vector3Stamped>("/accel/sample", 100);
        pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/camera/pose/sample", 100);

        // Configurazione dei flussi RealSense T265
        rs2::config cfg;
        cfg.enable_stream(RS2_STREAM_GYRO, RS2_FORMAT_MOTION_XYZ32F, 200);
        cfg.enable_stream(RS2_STREAM_ACCEL, RS2_FORMAT_MOTION_XYZ32F, 62);
        cfg.enable_stream(RS2_STREAM_POSE); // 200Hz di default per la posa 6DOF

        try {
            // Avvio della pipeline con callback asincrona
            pipe_.start(cfg, [this](rs2::frame frame) {
                process_frame(frame);
            });
            RCLCPP_INFO(this->get_logger(), "Pipeline RealSense T265 avviata correttamente.");
        } catch (const rs2::error & e) {
            RCLCPP_ERROR(this->get_logger(), "Errore RealSense: %s", e.what());
        }
    }

    ~T265BridgeNode() {
        try {
            pipe_.stop();
        } catch (...) {}
    }

private:
    void process_frame(const rs2::frame& frame) {
        auto current_time = this->now();

        // Gestione del flusso di Posa 6DOF
        if (rs2::pose_frame pose_frame = frame.as<rs2::pose_frame>()) {
            rs2_pose pose_data = pose_frame.get_pose_data();
            geometry_msgs::msg::PoseStamped pose_msg;
            
            pose_msg.header.stamp = current_time;
            pose_msg.header.frame_id = "camera_pose_link";

            // Posizione (Traslazione XYZ)
            pose_msg.pose.position.x = pose_data.translation.x;
            pose_msg.pose.position.y = pose_data.translation.y;
            pose_msg.pose.position.z = pose_data.translation.z;

            // Orientamento (Quaternione XYZW)
            pose_msg.pose.orientation.x = pose_data.rotation.x;
            pose_msg.pose.orientation.y = pose_data.rotation.y;
            pose_msg.pose.orientation.z = pose_data.rotation.z;
            pose_msg.pose.orientation.w = pose_data.rotation.w;

            pose_pub_->publish(pose_msg);
        }
        // Gestione dei flussi di Movimento (Gyro e Accel)
        else if (rs2::motion_frame motion_frame = frame.as<rs2::motion_frame>()) {
            rs2_vector motion_data = motion_frame.get_motion_data();
            geometry_msgs::msg::Vector3Stamped vec_msg;
            
            vec_msg.header.stamp = current_time;
            vec_msg.vector.x = motion_data.x;
            vec_msg.vector.y = motion_data.y;
            vec_msg.vector.z = motion_data.z;

            rs2_stream stream_type = motion_frame.get_profile().stream_type();
            if (stream_type == RS2_STREAM_GYRO) {
                vec_msg.header.frame_id = "camera_gyro_link";
                gyro_pub_->publish(vec_msg);
            } 
            else if (stream_type == RS2_STREAM_ACCEL) {
                vec_msg.header.frame_id = "camera_accel_link";
                accel_pub_->publish(vec_msg);
            }
        }
    }

    rs2::pipeline pipe_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr gyro_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr accel_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<T265BridgeNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}