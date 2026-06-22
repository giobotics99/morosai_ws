#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from optical_head.msg import PgvScanData
from sensor_msgs.msg import PointCloud2, PointField
import std_msgs.msg
import sensor_msgs_py.point_cloud2 as pc2
import math
import numpy as np

class VirtualCorridor(Node):
    def __init__(self):
        super().__init__('virtual_corridor_node')

        self.declare_parameter("corridor_width", 1.0)  # metri
        self.declare_parameter("corridor_length", 1.0)  # metri

        self.width = self.get_parameter("corridor_width").value
        self.length = self.get_parameter("corridor_length").value

        self.sub = self.create_subscription(
            PgvScanData,
            '/pgv100_scan',
            self.callback,
            10
        )

        self.pub = self.create_publisher(PointCloud2, '/virtual_corridor_cloud', 10)

    def callback(self, msg: PgvScanData):
        if msg.no_pos == 1:
            return  # niente tag → niente corridoio

        x = msg.x_pos / 1000.0  # posizione del tag (per riferimento)
        y = msg.y_pos / 1000.0  # centro corridoio su y
        points = []

        # genera due linee parallele come muri virtuali, lunghezza lungo X
        for dx in np.linspace(-self.length/2, self.length/2, int(self.length / 0.05)):
            yl = y - self.width  # lato sinistro
            yr = y + self.width  # lato destro
            xl = x - 2.5*self.length  # lato sinistro
            xr = x - 2.5*self.length  # lato destro
            # points.append([x + dx, yl, 0.0])
            # points.append([x + dx, yr, 0.0])
            points.append([xl + dx, yl, 0.0])
            points.append([xr + dx, yr, 0.0])

        self.pub.publish(self.create_cloud(points))

    def create_cloud(self, pts):
        header = std_msgs.msg.Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = 'base_footprint'  # come la costmap locale

        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]

        return pc2.create_cloud(header, fields, pts)

def main(args=None):
    rclpy.init(args=args)
    node = VirtualCorridor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
