#!/usr/bin/env python3
"""PGV100 fine-docking node for ROS 2 Jazzy.

During angular alignment, the AGV translates while rotating so that the
PGV-observed tag remains the instantaneous centre of rotation.
"""

from enum import Enum
import math

import rclpy
from action_msgs.srv import CancelGoal
from geometry_msgs.msg import PointStamped, Twist
from optical_head.msg import PgvScanData
from rclpy.node import Node
from std_msgs.msg import Bool, String
from tf2_ros import Buffer, TransformListener
from tf2_geometry_msgs import do_transform_point


class DockState(Enum):
    IDLE = 0
    DOCKING = 1
    DOCKED = 2


class PgvFloorDockNode(Node):
    def __init__(self):
        super().__init__('pgv_floor_dock_node')

        # Topics/services
        self.declare_parameter('pgv_topic', '/optical_head/pgv100_scan')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel_nav')
        self.declare_parameter(
            'cancel_service', '/navigate_to_pose/_action/cancel_goal'
        )
        self.declare_parameter('dock_command_topic', '/agv_dock')

        # PGV longitudinal position, in mm
        self.declare_parameter('x_start_mm', 4800.0)
        self.declare_parameter('x_stop_mm', 1150.0)
        self.declare_parameter('reverse_docking', True)

        # Limits
        self.declare_parameter('max_vx', 0.04)       # m/s
        self.declare_parameter('max_vy', 0.02)       # m/s
        self.declare_parameter('max_wz', 0.03)       # rad/s
        self.declare_parameter('max_accel_x', 0.03)  # m/s^2
        self.declare_parameter('max_accel_y', 0.02)  # m/s^2
        self.declare_parameter('max_accel_z', 0.03)  # rad/s^2

        # Proportional gains. kp_x and kp_y map mm to m/s;
        # kp_yaw maps degrees to rad/s.
        self.declare_parameter('kp_x', 0.00008)
        self.declare_parameter('kp_y', 0.0004)
        self.declare_parameter('kp_yaw', 0.0012)

        # Tolerances
        self.declare_parameter('deadband_y_mm', 1.0)
        self.declare_parameter('deadband_yaw_deg', 3.0)

        # Sign conventions retained from the original node
        self.declare_parameter('y_sign', 1.0)
        self.declare_parameter('angle_sign', -1.0)

        # Timing
        self.declare_parameter('scan_timeout', 0.30)
        self.declare_parameter('control_rate', 20.0)
        self.declare_parameter('max_allowed_missed_scans', 5)

        # TF frames. The xacro places optical_head_link at x=0.4375 m
        # and rotates it by pi around X with respect to base_link.
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('sensor_frame', 'optical_head_link')

        self.x_start_mm = float(self.get_parameter('x_start_mm').value)
        self.x_stop_mm = float(self.get_parameter('x_stop_mm').value)
        self.reverse_docking = bool(self.get_parameter('reverse_docking').value)

        self.max_vx = float(self.get_parameter('max_vx').value)
        self.max_vy = float(self.get_parameter('max_vy').value)
        self.max_wz = float(self.get_parameter('max_wz').value)
        self.max_accel_x = float(self.get_parameter('max_accel_x').value)
        self.max_accel_y = float(self.get_parameter('max_accel_y').value)
        self.max_accel_z = float(self.get_parameter('max_accel_z').value)

        self.kp_x = float(self.get_parameter('kp_x').value)
        self.kp_y = float(self.get_parameter('kp_y').value)
        self.kp_yaw = float(self.get_parameter('kp_yaw').value)
        self.deadband_y_mm = float(self.get_parameter('deadband_y_mm').value)
        self.deadband_yaw_deg = float(
            self.get_parameter('deadband_yaw_deg').value
        )
        self.y_sign = float(self.get_parameter('y_sign').value)
        self.angle_sign = float(self.get_parameter('angle_sign').value)
        self.scan_timeout = float(self.get_parameter('scan_timeout').value)
        self.control_rate = float(self.get_parameter('control_rate').value)
        if self.control_rate <= 0.0:
            raise ValueError('control_rate deve essere > 0')
        self.dt = 1.0 / self.control_rate
        self.max_allowed_missed_scans = int(
            self.get_parameter('max_allowed_missed_scans').value
        )

        self.base_frame = str(self.get_parameter('base_frame').value)
        self.sensor_frame = str(self.get_parameter('sensor_frame').value)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.dock_enabled = False
        self.state = DockState.IDLE
        self.latest_scan = None
        self.latest_scan_time = None
        self.consecutive_missed_scans = 0
        self.cancel_requested = False

        self.curr_vx = 0.0
        self.curr_vy = 0.0
        self.curr_wz = 0.0
        self.last_status = None

        self.cmd_pub = self.create_publisher(
            Twist, str(self.get_parameter('cmd_vel_topic').value), 10
        )
        self.status_pub = self.create_publisher(
            String, '/pgv_floor_dock/status', 10
        )
        self.dock_command_sub = self.create_subscription(
            Bool,
            str(self.get_parameter('dock_command_topic').value),
            self.dock_command_callback,
            10,
        )
        self.scan_sub = self.create_subscription(
            PgvScanData,
            str(self.get_parameter('pgv_topic').value),
            self.scan_callback,
            10,
        )
        self.cancel_client = self.create_client(
            CancelGoal, str(self.get_parameter('cancel_service').value)
        )
        self.timer = self.create_timer(self.dt, self.control_loop)

        self.get_logger().info(
            f'PGV docking pronto: x_start={self.x_start_mm:.1f} mm, '
            f'x_stop={self.x_stop_mm:.1f} mm, '
            f'base_frame={self.base_frame}, sensor_frame={self.sensor_frame}'
        )

    def publish_status(self, status):
        if status == self.last_status:
            return
        msg = String()
        msg.data = status
        self.status_pub.publish(msg)
        self.last_status = status

    def stop_robot(self):
        self.curr_vx = 0.0
        self.curr_vy = 0.0
        self.curr_wz = 0.0
        self.cmd_pub.publish(Twist())

    def is_valid_tag(self, msg):
        # Do not reject x_pos < x_stop_mm here: otherwise the success
        # condition would be unreachable when the robot crosses the stop.
        return msg.no_pos == 0 and not bool(msg.error)

    def is_scan_fresh(self):
        if self.latest_scan is None or self.latest_scan_time is None:
            return False
        age = (self.get_clock().now() - self.latest_scan_time).nanoseconds / 1e9
        return age <= self.scan_timeout

    def scan_callback(self, msg: PgvScanData):
        self.latest_scan = msg
        self.latest_scan_time = self.get_clock().now()

        if self.is_valid_tag(msg):
            self.consecutive_missed_scans = 0

        if not self.dock_enabled or self.state != DockState.IDLE:
            return

        if self.is_valid_tag(msg) and msg.x_pos <= self.x_start_mm:
            self.state = DockState.DOCKING
            self.cancel_requested = False
            self.publish_status('DOCKING_ACTIVE')
            self.request_nav_cancel()
            self.get_logger().info(
                f'Tag PGV agganciato: x={msg.x_pos:.1f} mm, '
                f'y={msg.y_pos:.1f} mm, angle={msg.angle:.1f} deg'
            )

    def dock_command_callback(self, msg: Bool):
        if not msg.data:
            self.dock_enabled = False
            self.state = DockState.IDLE
            self.latest_scan = None
            self.latest_scan_time = None
            self.consecutive_missed_scans = 0
            self.stop_robot()
            self.publish_status('DISABLED')
            return

        if self.dock_enabled and self.state != DockState.DOCKED:
            return

        self.dock_enabled = True
        self.state = DockState.IDLE
        self.latest_scan = None
        self.latest_scan_time = None
        self.consecutive_missed_scans = 0
        self.cancel_requested = False
        self.curr_vx = self.curr_vy = self.curr_wz = 0.0
        self.publish_status('ARMED')
        self.get_logger().info('PGV docking armato; in attesa di un tag')

    def angle_error_to_180(self, angle_deg):
        return (180.0 - angle_deg + 180.0) % 360.0 - 180.0

    def get_tag_xy_in_base(self, scan):
        """Return PGV tag point in base_link, in metres.

        scan.x_pos and scan.y_pos are assumed to be millimetres expressed in
        optical_head_link. No raw-coordinate fallback is used: on TF failure
        the caller stops instead of commanding motion with mixed frames.
        """
        point_sensor = PointStamped()
        point_sensor.header.frame_id = self.sensor_frame
        point_sensor.header.stamp = self.get_clock().now().to_msg()
        point_sensor.point.x = float(scan.x_pos) / 1000.0
        point_sensor.point.y = float(scan.y_pos) / 1000.0
        point_sensor.point.z = 0.0

        transform = self.tf_buffer.lookup_transform(
            self.base_frame,
            self.sensor_frame,
            rclpy.time.Time(),
        )
        point_base = do_transform_point(point_sensor, transform).point
        return float(point_base.x), float(point_base.y)

    def control_loop(self):
        if not self.dock_enabled or self.state in (DockState.IDLE, DockState.DOCKED):
            return

        if not self.is_scan_fresh() or not self.is_valid_tag(self.latest_scan):
            self.consecutive_missed_scans += 1
            if self.consecutive_missed_scans <= self.max_allowed_missed_scans:
                # Stop immediately on stale/lost data: do not keep applying
                # the last non-zero command while waiting for recovery.
                self.stop_robot()
                self.publish_status('WAITING_FOR_TAG')
                return

            self.stop_robot()
            self.publish_status('FAILED_TAG_LOST')
            self.get_logger().warning('Tag PGV perso; docking interrotto')
            self.dock_enabled = False
            self.state = DockState.IDLE
            return

        scan = self.latest_scan

        if scan.x_pos <= self.x_stop_mm:
            self.stop_robot()
            self.publish_status('SUCCESS')
            self.get_logger().info(
                f'Docking completato: x={scan.x_pos:.1f} mm'
            )
            self.dock_enabled = False
            self.state = DockState.DOCKED
            return

        try:
            self.compute_and_publish_cmd(scan)
        except Exception as exc:
            self.stop_robot()
            self.publish_status('TF_ERROR')
            self.get_logger().error(
                f'Impossibile trasformare la posizione del tag: {exc}. '
                'Comando di velocita arrestato.'
            )

    def apply_slew_rate(self, target, current, max_accel):
        max_delta = max_accel * self.dt
        delta = target - current
        if abs(delta) > max_delta:
            delta = math.copysign(max_delta, delta)
        return current + delta

    @staticmethod
    def clamp(value, limit):
        return max(-limit, min(limit, value))

    def compute_and_publish_cmd(self, scan):
        yaw_err_deg = self.angle_error_to_180(float(scan.angle))

        target_vx = 0.0
        target_vy = 0.0
        target_wz = 0.0

        if abs(yaw_err_deg) > self.deadband_yaw_deg:
            self.publish_status('ALIGNING_ROTATION')

            target_wz = self.clamp(
                self.kp_yaw * yaw_err_deg * self.angle_sign,
                self.max_wz,
            )

            # Pivot around the actual PGV-observed tag, not around base_link.
            # For a stationary point r=(x_tag,y_tag):
            # v_base = -omega x r = (omega*y_tag, -omega*x_tag).
            x_tag, y_tag = self.get_tag_xy_in_base(scan)
            target_vx = target_wz * y_tag
            target_vy = -target_wz * x_tag

            # Add a small lateral feedback term to remove residual calibration
            # and odometry errors. Both terms are in m/s after conversion.
            y_error_m = y_tag
            if abs(y_error_m * 1000.0) > self.deadband_y_mm:
                target_vy += self.y_sign * self.kp_y * (y_error_m * 1000.0)

            # Limit planar speed. Saturation means the pivot cannot be exact
            # at this angular speed; reduce omega if the requested correction
            # exceeds the available lateral/longitudinal velocity.
            target_vx = self.clamp(target_vx, self.max_vx)
            target_vy = self.clamp(target_vy, self.max_vy)

            self.get_logger().debug(
                f'ALIGN: yaw_err={yaw_err_deg:.2f} deg, '
                f'tag_base=({x_tag:.3f}, {y_tag:.3f}) m, '
                f'cmd=({target_vx:.4f}, {target_vy:.4f}, '
                f'{target_wz:.4f})'
            )

        else:
            self.publish_status('DOCKING_TRANSLATION')

            # Original PGV longitudinal docking controller.
            err_x_mm = float(scan.x_pos) - self.x_stop_mm
            vx_mag = self.kp_x * err_x_mm
            vx_mag = max(0.0, min(self.max_vx, vx_mag))
            target_vx = -vx_mag if self.reverse_docking else vx_mag

            # Correct lateral alignment using the tag position in base frame.
            _, y_tag = self.get_tag_xy_in_base(scan)
            y_error_mm = y_tag * 1000.0
            if abs(y_error_mm) > self.deadband_y_mm:
                target_vy = self.clamp(
                    self.y_sign * self.kp_y * y_error_mm, self.max_vy
                )

            target_wz = 0.0

        self.curr_vx = self.apply_slew_rate(
            target_vx, self.curr_vx, self.max_accel_x
        )
        self.curr_vy = self.apply_slew_rate(
            target_vy, self.curr_vy, self.max_accel_y
        )
        self.curr_wz = self.apply_slew_rate(
            target_wz, self.curr_wz, self.max_accel_z
        )

        cmd = Twist()
        cmd.linear.x = self.curr_vx
        cmd.linear.y = self.curr_vy
        cmd.angular.z = self.curr_wz
        self.cmd_pub.publish(cmd)

    def request_nav_cancel(self):
        if self.cancel_requested:
            return
        self.cancel_requested = True
        if not self.cancel_client.service_is_ready():
            self.get_logger().warning(
                'Servizio cancel goal non disponibile; il docking continua '
                'ma verificare che Nav2 non pubblichi contemporaneamente cmd_vel.'
            )
            return
        request = CancelGoal.Request()
        self.cancel_client.call_async(request)

    def destroy_node(self):
        self.stop_robot()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = PgvFloorDockNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
