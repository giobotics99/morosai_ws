#!/usr/bin/env python3

"""
MOROSAI AGV: Advanced Sensor Synchronization Monitor

This node monitors the synchronization health of the robot's primary sensors:
- LIDAR (/merged) - Primary SLAM/Navigation
- TOF Camera (/gordon_tof/pointcloud) - Primary Obstacle Avoidance
- PGV100 Sensor (/pgv100_scan) - Background Tactical Sensor

It synchronizes LIDAR and TOF (High Priority) while tracking PGV health independently.
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2, LaserScan
from optical_head.msg import PgvScanData
from std_msgs.msg import String
from message_filters import Subscriber, ApproximateTimeSynchronizer
import collections

class SensorHealth:
    def __init__(self):
        self.last_stamp = None
        self.last_arrival = None
        self.frequency = 0.0
        self.latency = 0.0
        self.arrival_history = collections.deque(maxlen=10)

    def update(self, stamp, now):
        current_arrival = now.nanoseconds / 1e9
        if self.last_arrival is not None:
            interval = current_arrival - self.last_arrival
            if interval > 0:
                self.arrival_history.append(interval)
                self.frequency = len(self.arrival_history) / sum(self.arrival_history)
        
        self.last_arrival = current_arrival
        self.last_stamp = stamp
        self.latency = (now - rclpy.time.Time.from_msg(stamp)).nanoseconds / 1e9

class AdvancedSensorSyncNode(Node):
    def __init__(self):
        super().__init__('sensor_sync_monitor')
        self.get_logger().info('Advanced Sensor Sync Monitor started')

        # ---- Parameters ----
        self.declare_parameter('merged_topic', '/merged')
        self.declare_parameter('tof_topic', '/gordon_tof/pointcloud')
        self.declare_parameter('pgv_topic', '/pgv100_scan')
        self.declare_parameter('queue_size', 30)
        self.declare_parameter('slop', 0.2)
        self.declare_parameter('status_period', 1.0)
        self.declare_parameter('max_latency_threshold', 0.5)

        merged_topic = self.get_parameter('merged_topic').value
        tof_topic = self.get_parameter('tof_topic').value
        pgv_topic = self.get_parameter('pgv_topic').value
        queue_size = self.get_parameter('queue_size').value
        self.slop = self.get_parameter('slop').value
        self.status_period = self.get_parameter('status_period').value
        self.max_latency = self.get_parameter('max_latency_threshold').value

        sensor_qos = qos_profile_sensor_data

        # ---- Health Trackers ----
        self.health = {
            'LIDAR': SensorHealth(),
            'TOF': SensorHealth(),
            'PGV': SensorHealth()
        }
        self.sync_count = 0
        self.last_sync_drift = 0.0

        # ---- Subscribers (Raw for independent tracking) ----
        self.create_subscription(LaserScan, merged_topic, lambda msg: self.health['LIDAR'].update(msg.header.stamp, self.get_clock().now()), sensor_qos)
        self.create_subscription(PointCloud2, tof_topic, lambda msg: self.health['TOF'].update(msg.header.stamp, self.get_clock().now()), sensor_qos)
        self.create_subscription(PgvScanData, pgv_topic, lambda msg: self.health['PGV'].update(msg.header.stamp, self.get_clock().now()), sensor_qos)

        # ---- Message Filters (For sync detection - LIDAR + TOF Only) ----
        # Decoupled PGV because it is a "background" sensor and shouldn't block the primary sync report
        self.merged_sub = Subscriber(self, LaserScan, merged_topic, qos_profile=sensor_qos)
        self.tof_sub = Subscriber(self, PointCloud2, tof_topic, qos_profile=sensor_qos)

        self.ts = ApproximateTimeSynchronizer(
            [self.merged_sub, self.tof_sub],
            queue_size=queue_size,
            slop=self.slop
        )
        self.ts.registerCallback(self.synced_callback)
        # ---- Publisher dello stato ----
        self.status_pub = self.create_publisher(String, '/sensor_sync/health_report', 10)

        # ---- Timer per report periodico ----
        self.create_timer(self.status_period, self.publish_report)

    def synced_callback(self, merged: LaserScan, tof: PointCloud2):
        self.sync_count += 1
        stamps = [
            rclpy.time.Time.from_msg(merged.header.stamp).nanoseconds,
            rclpy.time.Time.from_msg(tof.header.stamp).nanoseconds
        ]
        self.last_sync_drift = (max(stamps) - min(stamps)) / 1e9

    def publish_report(self):
        report = []
        report.append("="*50)
        report.append(f" MOROSAI SENSOR SYNC REPORT | Primary Syncs: {self.sync_count}")
        report.append("="*50)

        for name, data in self.health.items():
            status = "🟢 OK"
            if data.last_arrival is None:
                status = "🔴 MISSING"
            elif data.latency > self.max_latency:
                status = "🟡 LAGGY"
            elif data.frequency < 1.0:
                status = "🟡 SLOW/DROP"

            # Distinguish between Critical (LIDAR/TOF) and Optional (PGV)
            role = "(PRI)" if name in ['LIDAR', 'TOF'] else "(OPT)"
            report.append(f"[{status}] {name:5} {role} | Freq: {data.frequency:5.1f}Hz | Latency: {data.latency:6.3f}s")

        report.append("-" * 50)
        drift_status = "🟢 GOOD" if self.last_sync_drift < 0.1 else "🟡 HIGH"
        if self.sync_count == 0: drift_status = "🔴 NO SYNC"
        
        report.append(f"LIDAR-TOF Drift: {self.last_sync_drift:6.3f}s [{drift_status}]")
        report.append(f"Sync Slop (Threshold): {self.slop}s")
        report.append("=" * 50)

        msg = String()
        msg.data = "\n".join(report)
        self.status_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = AdvancedSensorSyncNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
