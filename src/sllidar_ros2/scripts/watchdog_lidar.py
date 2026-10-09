#!/usr/bin/env python3

import os
import signal
import subprocess
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from rclpy.qos import qos_profile_sensor_data
from rclpy.duration import Duration

class LidarWatchdogNode(Node):
    def __init__(self):
        super().__init__('lidar_watchdog_node')
        
        # Parametri timeout in secondi
        self.declare_parameter('timeout_sec', 3.0)
        self.timeout_sec = self.get_parameter('timeout_sec').value

        self.declare_parameter('grace_period', 15.0)
        grace = Duration(seconds=self.get_parameter('grace_period').value)

        now = self.get_clock().now()
        self.last_front_time = now + grace
        self.last_rear_time = now + grace

        # # Timestamps ultimo messaggio ricevuto
        # self.last_front_time = self.get_clock().now()
        # self.last_rear_time = self.get_clock().now()

        # Subscription con QoS SensorData (Best Effort, adatto ai LiDAR)
        self.sub_front = self.create_subscription(
            LaserScan, '/lidar_front/scan', self.cb_front, qos_profile_sensor_data)
        self.sub_rear = self.create_subscription(
            LaserScan, '/lidar_rear/scan', self.cb_rear, qos_profile_sensor_data)

        # Timer di verifica a 1 Hz
        self.timer = self.create_timer(1.0, self.check_health)
        self.get_logger().info("Lidar Watchdog attivo e in ascolto sui topic dei sensori.")

    def cb_front(self, msg):
        self.last_front_time = self.get_clock().now()

    def cb_rear(self, msg):
        self.last_rear_time = self.get_clock().now()

    def kill_node_by_namespace(self, ns_name):
        """Trova il PID del sllidar_node che gira in un determinato namespace/nome e invia SIGKILL."""
        try:
            output = subprocess.check_output(["pgrep", "-f", "sllidar_node"]).decode().split()
            for pid_str in output:
                pid = int(pid_str)
                with open(f"/proc/{pid}/cmdline", "r") as f:
                    cmd = f.read()
                    if ns_name in cmd:
                        self.get_logger().error(
                            f"HARDWARE TIMEOUT: Termino il processo zombie {ns_name} (PID {pid}) per forzare il respawn!"
                        )
                        os.kill(pid, signal.SIGKILL)
        except Exception as e:
            self.get_logger().warn(f"Impossibile terminare il processo per {ns_name}: {e}")

    def check_health(self):
        now = self.get_clock().now()
        
        # Check Front Lidar
        dt_front = (now - self.last_front_time).nanoseconds / 1e9
        if dt_front > self.timeout_sec:
            self.get_logger().error(f"LiDAR Frontale KO! Nessun dato da {dt_front:.1f}s.")
            self.kill_node_by_namespace('lidar_front')
            # Reset del timer per evitare chiamate a raffica sullo stesso processo
            # self.last_front_time = now
            grace = Duration(seconds=self.get_parameter('grace_period').value)
            self.last_front_time = now + grace

        # Check Rear Lidar
        dt_rear = (now - self.last_rear_time).nanoseconds / 1e9
        if dt_rear > self.timeout_sec:
            self.get_logger().error(f"LiDAR Posteriore KO! Nessun dato da {dt_rear:.1f}s.")
            self.kill_node_by_namespace('lidar_rear')
            # Reset del timer per evitare chiamate a raffica sullo stesso processo
            # self.last_rear_time = now
            grace = Duration(seconds=self.get_parameter('grace_period').value)
            self.last_rear_time = now + grace


def main(args=None):
    rclpy.init(args=args)
    node = LidarWatchdogNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()