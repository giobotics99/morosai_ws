#!/usr/bin/env python3

import subprocess
import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty

class JetsonShutdownNode(Node):
    def __init__(self):
        super().__init__('jetson_shutdown_node')
        
        # Parametro o topic fisso per lo spegnimento
        self.declare_parameter('shutdown_topic', '/shutdown')
        self.shutdown_topic = self.get_parameter('shutdown_topic').get_parameter_value().string_value
        
        # Sottoscrizione al topic di shutdown
        self.create_subscription(Empty, self.shutdown_topic, self.shutdown_callback, 10)
        self.get_logger().info(f"Jetson shutdown node avviato, in ascolto su {self.shutdown_topic}")

    def shutdown_callback(self, msg):
        self.get_logger().warning("Ricevuto comando di spegnimento sul topic ROS 2. Spegnimento della Jetson in corso...")
        # Esegue il comando di spegnimento di sistema
        subprocess.run(['sudo', 'shutdown', 'now'])

def main(args=None):
    rclpy.init(args=args)
    node = JetsonShutdownNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()