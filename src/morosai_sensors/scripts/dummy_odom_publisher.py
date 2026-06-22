#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
import tf2_ros
import math
import time


class DummyOdom(Node):
    def __init__(self):
        super().__init__("dummy_odom_publisher")

        self.declare_parameter("speed", 0.2)  # m/s
        self.speed = self.get_parameter("speed").value
        self.declare_parameter("moving", False)
        self.moving = self.get_parameter("moving").value

        self.odom_pub = self.create_publisher(Odometry, "/odom", 10)

        # TF broadcaster odom -> base_footprint
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        self.last_time = self.get_clock().now()

        self.timer = self.create_timer(0.05, self.timer_callback)  # 20 Hz

    def timer_callback(self):
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds * 1e-9
        self.last_time = now

        # Movimento in avanti
        if self.moving:
            self.x += self.speed * dt

        # -------- TF odom -> base_footprint --------
        t = TransformStamped()
        t.header.stamp = now.to_msg()
        t.header.frame_id = "odom"
        t.child_frame_id = "base_footprint"

        t.transform.translation.x = float(self.x)
        t.transform.translation.y = float(self.y)
        t.transform.translation.z = 0.0

        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0

        self.tf_broadcaster.sendTransform(t)

        # -------- Messaggio Odometry --------
        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_footprint"

        odom.pose.pose.position.x = float(self.x)
        odom.pose.pose.position.y = float(self.y)
        odom.pose.pose.position.z = 0.0

        odom.pose.pose.orientation.w = 1.0

        if self.moving:
            odom.twist.twist.linear.x = self.speed
        else:
            odom.twist.twist.linear.x = 0.0
        odom.twist.twist.angular.z = 0.0

        self.odom_pub.publish(odom)


def main(args=None):
    rclpy.init(args=args)
    node = DummyOdom()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()