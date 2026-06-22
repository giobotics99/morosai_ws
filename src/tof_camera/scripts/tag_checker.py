#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from apriltag_msgs.msg import AprilTagDetectionArray
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2

class TagChecker(Node):
    def __init__(self):
        super().__init__('tag_checker')
        self.create_subscription(AprilTagDetectionArray, '/detections', self.cb_tags, 10)
        self.create_subscription(PointCloud2, '/gordon_tof/pointcloud', self.cb_pc, 10)
        self.latest_pc = None
        self.pc_msg = None

    def cb_pc(self, msg):
        self.pc_msg = msg
        self.latest_pc = list(pc2.read_points(msg, field_names=('x','y','z'), skip_nans=False))

    def cb_tags(self, msg):
        if self.latest_pc is None:
            return
        width = self.pc_msg.width
        height = self.pc_msg.height
        for det in msg.detections:
            u, v = int(det.centre.x), int(det.centre.y)
            if 0 <= u < width and 0 <= v < height:
                # calcola indice lineare
                idx = v * width + u
                if idx < len(self.latest_pc):
                    x, y, z = self.latest_pc[idx]
                    print(f"Tag {det.id} a pixel ({u},{v}) → Z: {z:.3f} m, X: {x:.3f}, Y: {y:.3f}")
                else:
                    print(f"Tag {det.id} pixel ({u},{v}) indice fuori bounds")
            else:
                print(f"Tag {det.id} pixel ({u},{v}) fuori range")

def main():
    rclpy.init()
    node = TagChecker()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()