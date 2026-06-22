#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image
import yaml
import sys

class StaticCameraInfoPublisher(Node):
    def __init__(self, yaml_path):
        super().__init__('static_camera_info_publisher')

        self.publisher_ = self.create_publisher(CameraInfo, '/gordon_tof/camera_info', 10)

        # Carica YAML
        with open(yaml_path, 'r') as f:
            data = yaml.safe_load(f)

        self.cam_info = CameraInfo()
        self.cam_info.width = data['image_width']
        self.cam_info.height = data['image_height']
        self.cam_info.k = data['camera_matrix']['data']
        self.cam_info.d = data['distortion_coefficients']['data']
        self.cam_info.r = data['rectification_matrix']['data']
        self.cam_info.p = data['projection_matrix']['data']
        self.cam_info.distortion_model = data['distortion_model']
        self.cam_info.header.frame_id = 'tof_link'  # Assicurati che questo corrisponda al frame_id usato per l'immagine

        # Subscriber all'immagine BGR
        self.sub = self.create_subscription(
            Image,
            '/gordon_tof/bgr',
            self.image_callback,
            10
        )

        self.get_logger().info(f'Pubblicazione CameraInfo sincronizzata con /gordon_tof/bgr da {yaml_path}')

    def image_callback(self, img_msg):
        # Aggiorna header con lo stesso timestamp dell'immagine
        self.cam_info.header.stamp = img_msg.header.stamp
        self.publisher_.publish(self.cam_info)

def main(args=None):
    if len(sys.argv) < 2:
        print("Uso: python3 camera_info_publisher.py <percorso_yaml>")
        return

    rclpy.init(args=args)
    node = StaticCameraInfoPublisher(sys.argv[1])
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
