#!/usr/bin/env python3

from enum import Enum
import rclpy
from action_msgs.srv import CancelGoal
from geometry_msgs.msg import Twist
from optical_head.msg import PgvScanData
from rclpy.node import Node
from std_msgs.msg import Bool, String


class DockState(Enum):
    IDLE = 0
    DOCKING = 1
    DOCKED = 2


class PgvFloorDockNode(Node):
    """Nodo di docking olonomico reattivo con rampa di accelerazione e velocità contenute per PGV."""

    def __init__(self):
        super().__init__('pgv_floor_dock_node')

        # Topic e parametri generali
        self.declare_parameter('pgv_topic', '/optical_head/pgv100_scan')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('cancel_service', '/navigate_to_pose/_action/cancel_goal')
        self.declare_parameter('dock_command_topic', '/agv_dock')

        # Quota X di partenza e arresto (in mm)
        self.declare_parameter('x_start_mm', 4800.0)
        self.declare_parameter('x_stop_mm', 1150.0)
        self.declare_parameter('reverse_docking', True)

        # VELOCITÀ MASSIME MOLTO MOLTO DOLCI (in m/s e rad/s)
        self.declare_parameter('max_vx', 0.04)        # Max 4 cm/s in retromarcia
        self.declare_parameter('max_vy', 0.02)        # Max 2 cm/s di traslazione
        self.declare_parameter('max_wz', 0.03)        # Max 0.03 rad/s di rotazione

        # RAMPA DI ACCELERAZIONE MASSIMA (m/s² e rad/s²)
        self.declare_parameter('max_accel_x', 0.03)   # Incremento max di velocità x al secondo
        self.declare_parameter('max_accel_y', 0.02)   # Incremento max di velocità y al secondo
        self.declare_parameter('max_accel_z', 0.03)   # Incremento max di velocità z al secondo

        # GUADAGNI PROPORZIONALI AMMORBITI
        self.declare_parameter('kp_x', 0.00008)
        self.declare_parameter('kp_y', 0.0004)
        self.declare_parameter('kp_yaw', 0.0012)

        # TOLLERANZE / ZONA MORTA (Evita scatti per micro-oscillazioni)
        self.declare_parameter('deadband_y_mm', 5.0)
        self.declare_parameter('deadband_yaw_deg', 1.0)

        # Segni dei guadagni (invertire se corregge al contrario)
        self.declare_parameter('y_sign', -1.0)
        self.declare_parameter('angle_sign', 1.0)

        # Timeout e frequenza di controllo
        self.declare_parameter('scan_timeout', 0.30)
        self.declare_parameter('control_rate', 20.0)

        # Distanza in metri dal centro di rotazione dell'AGV al sensore PGV sul pavimento
        self.declare_parameter('sensor_offset_x', 0.43720)
        self.sensor_offset_x = float(self.get_parameter('sensor_offset_x').value)

        # Tolleranza ai micro-drop del sensore (es. tollera fino a 5 frame persi di fila ~ 0.25s)
        self.consecutive_missed_scans = 0
        self.max_allowed_missed_scans = 5

        # Lettura parametri
        pgv_topic = self.get_parameter('pgv_topic').value
        cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        cancel_service = self.get_parameter('cancel_service').value
        dock_command_topic = self.get_parameter('dock_command_topic').value

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
        self.deadband_yaw_deg = float(self.get_parameter('deadband_yaw_deg').value)

        self.y_sign = float(self.get_parameter('y_sign').value)
        self.angle_sign = float(self.get_parameter('angle_sign').value)
        self.scan_timeout = float(self.get_parameter('scan_timeout').value)
        self.dt = 1.0 / float(self.get_parameter('control_rate').value)

        # Stato interno e velocità correnti per la rampa
        self.dock_enabled = False
        self.state = DockState.IDLE
        self.latest_scan = None
        self.latest_scan_time = None
        self.cancel_requested = False

        self.curr_vx = 0.0
        self.curr_vy = 0.0
        self.curr_wz = 0.0

        # Publisher e Subscriber
        self.cmd_pub = self.create_publisher(Twist, cmd_vel_topic, 10)
        self.status_pub = self.create_publisher(String, '/pgv_floor_dock/status', 10)

        self.dock_command_sub = self.create_subscription(
            Bool, dock_command_topic, self.dock_command_callback, 10
        )
        self.scan_sub = self.create_subscription(
            PgvScanData, pgv_topic, self.scan_callback, 10
        )

        self.cancel_client = self.create_client(CancelGoal, cancel_service)

        self.timer = self.create_timer(self.dt, self.control_loop)

        self.get_logger().info(
            f'PGV Docking dolce avviato: x={self.x_start_mm:.0f} -> {self.x_stop_mm:.0f} mm (Max v={self.max_vx} m/s)'
        )

    def scan_callback(self, msg: PgvScanData):
        self.latest_scan = msg
        self.latest_scan_time = self.get_clock().now()

        # Se il dato e valido, resetta il contatore dei drop
        if self.is_valid_tag(msg):
            self.consecutive_missed_scans = 0

        if not self.dock_enabled or self.state != DockState.IDLE:
            return

        if self.is_valid_tag(msg):
            self.state = DockState.DOCKING
            self.cancel_requested = False
            self.publish_status('DOCKING_ACTIVE')
            self.request_nav_cancel()
            self.get_logger().info(
                f'Tag PGV agganciato a x={msg.x_pos:.1f} mm, y={msg.y_pos:.1f} mm, angle={msg.angle:.1f} deg'
            )

    def dock_command_callback(self, msg: Bool):
        if not msg.data:
            self.dock_enabled = False
            self.stop_robot()
            self.state = DockState.IDLE
            self.latest_scan = None
            self.latest_scan_time = None
            self.publish_status('DISABLED')
            return

        if self.dock_enabled and self.state != DockState.DOCKED:
            return

        self.dock_enabled = True
        self.state = DockState.IDLE
        self.latest_scan = None
        self.latest_scan_time = None
        self.curr_vx = 0.0
        self.curr_vy = 0.0
        self.curr_wz = 0.0
        self.publish_status('ARMED')
        self.get_logger().info('PGV docking armato; in attesa di un tag')

    def is_valid_tag(self, msg: PgvScanData):
        return (
            msg.no_pos == 0
            and self.x_stop_mm <= msg.x_pos <= self.x_start_mm
            and msg.error is False
        )

    def angle_error(self, angle_deg):
        angle_to_zero = (0.0 - angle_deg + 180.0) % 360.0 - 180.0
        angle_to_180 = (180.0 - angle_deg + 180.0) % 360.0 - 180.0
        return angle_to_zero if abs(angle_to_zero) <= abs(angle_to_180) else angle_to_180

    # def control_loop(self):
    #     if not self.dock_enabled or self.state in (DockState.IDLE, DockState.DOCKED):
    #         return

    #     if self.latest_scan is None or self.latest_scan_time is None:
    #         self.stop_robot()
    #         return

    #     age = (self.get_clock().now() - self.latest_scan_time).nanoseconds / 1e9
    #     if age > self.scan_timeout or not self.is_valid_tag(self.latest_scan):
    #         self.stop_robot()
    #         self.publish_status('FAILED_TAG_LOST')
    #         self.get_logger().warn('Tag PGV perso o non valido; docking interrotto')
    #         self.dock_enabled = False
    #         self.state = DockState.IDLE
    #         return

    #     scan = self.latest_scan

    #     # Condizione di arrivo
    #     if scan.x_pos <= self.x_stop_mm:
    #         self.stop_robot()
    #         self.publish_status('SUCCESS')
    #         self.get_logger().info(f'Docking completato a x={scan.x_pos:.1f} mm')
    #         self.dock_enabled = False
    #         self.state = DockState.DOCKED
    #         return

    #     # Calcola le velocità desiderate in modo graduale
    #     self.compute_and_publish_cmd(scan.x_pos, scan.y_pos, scan.angle)

    def control_loop(self):
        if not self.dock_enabled or self.state in (DockState.IDLE, DockState.DOCKED):
            return

        # Verifichiamo l'eta del dato o la validita
        is_stale = False
        if self.latest_scan_time is not None:
            age = (self.get_clock().now() - self.latest_scan_time).nanoseconds / 1e9
            if age > self.scan_timeout:
                is_stale = True

        if self.latest_scan is None or is_stale or not self.is_valid_tag(self.latest_scan):
            self.consecutive_missed_scans += 1
            # Se ha perso solo 1 o 2 frame, MANTIENE L'ULTIMO COMANDO o rallenta invece di disarmarsi subito!
            if self.consecutive_missed_scans <= self.max_allowed_missed_scans:
                self.get_logger().warn(f'Micro-drop PGV rilevato ({self.consecutive_missed_scans}/{self.max_allowed_missed_scans}), proseguo...')
                return
            else:
                # Solo se il tag e veramente perso per piu di 5 cicli consecutivi ferma tutto
                self.stop_robot()
                self.publish_status('FAILED_TAG_LOST')
                self.get_logger().warn('Tag PGV perso definitivamente; docking interrotto')
                self.dock_enabled = False
                self.state = DockState.IDLE
                return

        scan = self.latest_scan

        # Condizione di arrivo a destinazione
        if scan.x_pos <= self.x_stop_mm:
            self.stop_robot()
            self.publish_status('SUCCESS')
            self.get_logger().info(f'Docking completato con successo a x={scan.x_pos:.1f} mm')
            self.dock_enabled = False
            self.state = DockState.DOCKED
            return

        # Esegue il comando di retromarcia e centraggio
        self.compute_and_publish_cmd(scan.x_pos, scan.y_pos, scan.angle)

    def apply_slew_rate(self, target, current, max_accel):
        """Limita la variazione istantanea di velocità per evitare scatti."""
        max_delta = max_accel * self.dt
        delta = target - current
        if abs(delta) > max_delta:
            delta = max_delta if delta > 0 else -max_delta
        return current + delta
    
    def compute_and_publish_cmd(self, x_pos_mm, y_pos_mm, angle_deg):
        # 1. RETROMARCIA X (Indietreggia in modo continuo)
        err_x = x_pos_mm - self.x_stop_mm
        target_vx_mag = self.kp_x * err_x
        # Velocità minima garantita di 2.0 cm/s
        target_vx_mag = max(0.02, min(self.max_vx, target_vx_mag))
        target_vx = -target_vx_mag if self.reverse_docking else target_vx_mag

        # 2. TRASLAZIONE LATERALE Y (Segno invertito per centrare y_pos)
        # Forza y_sign a -1.0 se spinge dalla parte sbagliata
        effective_y_sign = -1.0  # Invertito rispetto a prima!

        if abs(y_pos_mm) <= 1.0:
            target_vy = 0.0
        else:
            # Controllo proporzionale diretto per riportare y_pos verso 0
            target_vy = effective_y_sign * self.kp_y * y_pos_mm
            target_vy = max(-self.max_vy, min(self.max_vy, target_vy))

        # 3. ROTAZIONE Z BLOCCATA A ZERO
        # L'angolo a 17.8 deg e perfetto: NON applicare alcuna rotazione!
        target_wz = 0.0

        # 4. RAMPA DI ACCELERAZIONE
        self.curr_vx = self.apply_slew_rate(target_vx, self.curr_vx, self.max_accel_x)
        self.curr_vy = self.apply_slew_rate(target_vy, self.curr_vy, self.max_accel_y)
        self.curr_wz = 0.0

        # Pubblicazione comandi al robot
        twist = Twist()
        twist.linear.x = self.curr_vx
        twist.linear.y = self.curr_vy
        twist.angular.z = 0.0

        self.cmd_pub.publish(twist)

    def request_nav_cancel(self):
        if self.cancel_requested:
            return
        self.cancel_requested = True
        if not self.cancel_client.service_is_ready():
            return
        request = CancelGoal.Request()
        self.cancel_client.call_async(request)

    def stop_robot(self):
        self.curr_vx = 0.0
        self.curr_vy = 0.0
        self.curr_wz = 0.0
        self.cmd_pub.publish(Twist())

    def publish_status(self, status):
        msg = String()
        msg.data = status
        self.status_pub.publish(msg)

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