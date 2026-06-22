#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, PointCloud2
from laser_geometry import LaserProjection

class DualScanToCloud(Node):
    def __init__(self):
        super().__init__('dual_scan_to_cloud')

        self.lp = LaserProjection()

        # front
        self.sub_front = self.create_subscription(
            LaserScan,
            '/lidar_front/scan',
            self.cb_front,
            10
        )
        self.pub_front = self.create_publisher(PointCloud2, '/lidar_front/points', 10)

        # rear
        self.sub_rear = self.create_subscription(
            LaserScan,
            '/lidar_rear/scan',
            self.cb_rear,
            10
        )
        self.pub_rear = self.create_publisher(PointCloud2, '/lidar_rear/points', 10)

        self.get_logger().info("Listening to /lidar_front/scan and /lidar_rear/scan")

    def cb_front(self, scan: LaserScan):
        cloud = self.lp.projectLaser(scan)
        self.pub_front.publish(cloud)

    def cb_rear(self, scan: LaserScan):
        cloud = self.lp.projectLaser(scan)
        self.pub_rear.publish(cloud)

def main():
    rclpy.init()
    node = DualScanToCloud()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
