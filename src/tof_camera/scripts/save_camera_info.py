#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo
import yaml
import os

class CameraInfoSaver(Node):
    def __init__(self):
        super().__init__('camera_info_saver')
        self.subscription = self.create_subscription(
            CameraInfo,
            '/gordon_tof/camera_info',
            self.listener_callback,
            10)
        self.get_logger().info('In attesa di un messaggio CameraInfo su /gordon_tof/camera_info...')
        self.saved = False

    def listener_callback(self, msg):
        if self.saved:
            return

        data = {
            'image_width': msg.width,
            'image_height': msg.height,
            'camera_name': 'gordon_tof',
            'camera_matrix': {
                'rows': 3,
                'cols': 3,
                'data': [float(x) for x in msg.k]
            },
            'distortion_model': msg.distortion_model,
            'distortion_coefficients': {
                'rows': 1,
                'cols': len(msg.d),
                'data': [float(x) for x in msg.d]
            },
            'rectification_matrix': {
                'rows': 3,
                'cols': 3,
                'data': [float(x) for x in msg.r]
            },
            'projection_matrix': {
                'rows': 3,
                'cols': 4,
                'data': [float(x) for x in msg.p]
            }
        }

        file_path = os.path.join(os.getcwd(), 'tof_calib.yaml')
        with open(file_path, 'w') as f:
            yaml.safe_dump(data, f, default_flow_style=False)

        self.get_logger().info(f'Calibrazione salvata in: {file_path}')
        self.saved = True
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    saver = CameraInfoSaver()
    rclpy.spin(saver)

if __name__ == '__main__':
    main()
