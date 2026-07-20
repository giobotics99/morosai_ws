#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener
import tf2_ros
import math
import numpy as np
import time

def quat_to_rot_matrix(q):
    qx, qy, qz, qw = q
    return np.array([
        [1 - 2*qy**2 - 2*qz**2,     2*qx*qy - 2*qz*qw,     2*qx*qz + 2*qy*qw],
        [    2*qx*qy + 2*qz*qw, 1 - 2*qx**2 - 2*qz**2,     2*qy*qz - 2*qx*qw],
        [    2*qx*qz - 2*qy*qw,     2*qy*qz + 2*qx*qw, 1 - 2*qx**2 - 2*qy**2]
    ])

class TagDockAlignNode(Node):
    def __init__(self):
        super().__init__('tag_dock_align_node')
        
        # --- PARAMETRI ---
        self.declare_parameter('allowed_dock_tags', ['tagStandard52h13:150'])
        self.declare_parameter('camera_frame', 'tof_optical_frame')
        self.declare_parameter('robot_frame', 'base_footprint')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel_raw')
        # Target distance: robot stops 1.0m from the tag (along camera Z)
        self.declare_parameter('target_distance', 0.60)
        # Alignment axis: which tag in-plane axis defines the approach direction
        # 'x' = tag X-axis, 'y' = tag Y-axis (check in RViz which one is correct)
        self.declare_parameter('alignment_axis', 'y')
        # Flip the sign of the alignment axis if it points away from the robot
        self.declare_parameter('flip_alignment_axis', False)
        # PID gains — lower for docking precision
        self.declare_parameter('kp_x', 0.5)
        self.declare_parameter('kp_y', 0.5)
        self.declare_parameter('kp_yaw', 0.8)
        # Slower speeds for docking
        self.declare_parameter('max_linear_vel', 0.05)
        self.declare_parameter('max_angular_vel', 0.03)
        # Tighter tolerances for docking
        self.declare_parameter('xy_tolerance', 0.03)
        self.declare_parameter('yaw_tolerance', 0.04)
        self.declare_parameter('alignment_timeout', 20.0)
        
        self.allowed_tags = self.get_parameter('allowed_dock_tags').value
        self.camera_frame = self.get_parameter('camera_frame').value
        self.robot_frame = self.get_parameter('robot_frame').value
        self.target_distance = self.get_parameter('target_distance').value
        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        self.alignment_axis = self.get_parameter('alignment_axis').value
        self.flip_axis = self.get_parameter('flip_alignment_axis').value
        self.kp_x = self.get_parameter('kp_x').value
        self.kp_y = self.get_parameter('kp_y').value
        self.kp_yaw = self.get_parameter('kp_yaw').value
        self.max_lin_vel = self.get_parameter('max_linear_vel').value
        self.max_ang_vel = self.get_parameter('max_angular_vel').value
        self.xy_tol = self.get_parameter('xy_tolerance').value
        self.yaw_tol = self.get_parameter('yaw_tolerance').value
        self.alignment_timeout = self.get_parameter('alignment_timeout').value
        
        # --- STATO INTERNO ---
        self.target_tag_frame = ""
        self.active_alignment = False
        self.start_time = None
        
        # Sicurezza collisioni
        self.is_paused = False
        self.pause_start_time = 0.0
        self.safety_wait_duration = 1.0  
        self.last_sent_linear_x = 0.0
        
        # TF2 Setup
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # --- PUB/SUB ROS 2 ---
        self.cmd_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.result_pub = self.create_publisher(String, '/dock_align_result', 10)
        
        # Comando di attivazione (separato dal servizio di allineamento macchina)
        self.command_sub = self.create_subscription(String, '/agv_dock_align', self.command_callback, 10)
        self.safety_sub = self.create_subscription(Twist, '/cmd_vel', self.safety_feedback_callback, 10)
        
        # Loop di controllo a 10Hz
        self.timer = self.create_timer(0.1, self.control_loop)
        
        self.get_logger().info(
            f"Nodo Dock Aligner avviato in IDLE. "
            f"Asse di allineamento: tag_{self.alignment_axis}, "
            f"distanza target: {self.target_distance}m"
        )

    def safety_feedback_callback(self, msg):
        current_time = time.time()
        if abs(self.last_sent_linear_x) > 0.01 and abs(msg.linear.x) < 0.001:
            if not self.is_paused:
                self.is_paused = True
                self.pause_start_time = current_time
                self.get_logger().error("COLLISION MONITOR INTERVENUTO!")

    def command_callback(self, msg):
        """Riceve il comando dallo scheduler per il docking."""
        requested_tag = msg.data
        self.get_logger().info(f"[DOCK] Ricevuto comando per tag: {requested_tag}")
        
        # Aggiorna al volo i parametri
        self.allowed_tags = self.get_parameter('allowed_dock_tags').value
        
        if requested_tag not in self.allowed_tags:
            self.get_logger().error(f"[DOCK] Tag {requested_tag} NON consentito per docking.")
            self.publish_result("REJECTED_NOT_ALLOWED")
            return
            
        # Controllo visibilità istantanea
        try:
            self.tf_buffer.lookup_transform(
                self.camera_frame, requested_tag, rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.1)
            )
            self.target_tag_frame = requested_tag
            self.start_time = self.get_clock().now()
            self.active_alignment = True
            self.get_logger().info(f"[DOCK] Tag {requested_tag} agganciato. Avvio allineamento docking.")
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            self.get_logger().warn(f"[DOCK] Tag {requested_tag} consentito ma NON visibile.")
            self.publish_result("REJECTED_NOT_VISIBLE")

    def control_loop(self):
        if not self.active_alignment:
            return

        # Timeout di sicurezza
        elapsed = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
        if elapsed > self.alignment_timeout:
            self.get_logger().error("[DOCK] TIMEOUT SCADUTO. Allineamento docking fallito.")
            self.stop_robot()
            self.active_alignment = False
            self.publish_result("FAILED_TIMEOUT")
            return

        current_time = time.time()
        if self.is_paused:
            if (current_time - self.pause_start_time) < self.safety_wait_duration:
                self.stop_robot()
                return
            else:
                self.is_paused = False

        try:
            t_cam_tag = self.tf_buffer.lookup_transform(self.camera_frame, self.target_tag_frame, rclpy.time.Time())
            t_base_cam = self.tf_buffer.lookup_transform(self.robot_frame, self.camera_frame, rclpy.time.Time())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            self.stop_robot()
            return
            
        # =============================================
        # CALCOLO ERRORI PER TAG SUL PAVIMENTO
        # =============================================
        # 
        # Differenza chiave rispetto al tag a muro:
        # - Tag Z punta VERSO L'ALTO (fuori dal pavimento), perpendicolare alla camera Z
        # - Per lo yaw, usiamo l'asse X o Y del tag (quelli nel piano del pavimento)
        #   proiettati sul piano XZ della camera
        #
        
        # Errori di posizione (identici al caso a muro)
        x = t_cam_tag.transform.translation.x
        z = t_cam_tag.transform.translation.z
        error_lat = x           
        error_fwd = z - self.target_distance  
        
        # Errore di yaw — DIVERSO dal caso a muro!
        # Usiamo l'asse del tag che giace nel piano del pavimento
        q_tag = [
            t_cam_tag.transform.rotation.x,
            t_cam_tag.transform.rotation.y,
            t_cam_tag.transform.rotation.z,
            t_cam_tag.transform.rotation.w
        ]
        R_cam_tag = quat_to_rot_matrix(q_tag)
        
        # Seleziona l'asse del tag nel piano del pavimento per l'allineamento
        if self.alignment_axis == 'x':
            align_axis = R_cam_tag @ np.array([1, 0, 0])
        else:  # 'y'
            align_axis = R_cam_tag @ np.array([0, 1, 0])
        
        if self.flip_axis:
            align_axis = -align_axis
        
        # Proietta l'asse scelto sul piano XZ della camera per ottenere lo yaw
        # atan2(componente_x, componente_z) → angolo nel piano orizzontale della camera
        error_yaw = math.atan2(align_axis[0], align_axis[2])
        
        self.get_logger().info(
            f"[DOCK] err_lat={error_lat:.3f} err_fwd={error_fwd:.3f} "
            f"err_yaw={math.degrees(error_yaw):.1f}° "
            f"(axis_{self.alignment_axis} in cam: [{align_axis[0]:.2f}, {align_axis[1]:.2f}, {align_axis[2]:.2f}])",
            throttle_duration_sec=0.5
        )
        
        # Condizione di convergenza
        if abs(error_lat) <= self.xy_tol and abs(error_fwd) <= self.xy_tol and abs(error_yaw) <= self.yaw_tol:
            self.get_logger().info("[DOCK] ALLINEAMENTO DOCKING COMPLETATO CON SUCCESSO.")
            self.stop_robot()
            self.active_alignment = False
            self.publish_result("SUCCESS")
            return
        
        # Calcolo velocità in camera frame
        v_cam_x = self.kp_x * error_lat if abs(error_lat) > self.xy_tol else 0.0
        v_cam_z = self.kp_y * error_fwd if abs(error_fwd) > self.xy_tol else 0.0
        w_cam_y = self.kp_yaw * error_yaw if abs(error_yaw) > self.yaw_tol else 0.0
        
        v_cam_x = max(-self.max_lin_vel, min(self.max_lin_vel, v_cam_x))
        v_cam_z = max(-self.max_lin_vel, min(self.max_lin_vel, v_cam_z))
        w_cam_y = max(-self.max_ang_vel, min(self.max_ang_vel, w_cam_y))
        
        # Trasformazione nel frame robot
        q_base_cam = [
            t_base_cam.transform.rotation.x,
            t_base_cam.transform.rotation.y,
            t_base_cam.transform.rotation.z,
            t_base_cam.transform.rotation.w
        ]
        R_base_cam = quat_to_rot_matrix(q_base_cam)
        V_base = R_base_cam @ np.array([v_cam_x, 0.0, v_cam_z])
        W_base = R_base_cam @ np.array([0.0, w_cam_y, 0.0])
        
        twist = Twist()
        twist.linear.x = float(V_base[0])
        twist.linear.y = float(V_base[1])
        twist.angular.z = float(W_base[2])
        
        self.last_sent_linear_x = twist.linear.x
        self.cmd_pub.publish(twist)

    def stop_robot(self):
        twist = Twist()
        self.last_sent_linear_x = 0.0
        self.cmd_pub.publish(twist)

    def publish_result(self, status_string):
        msg = String()
        msg.data = status_string
        self.result_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = TagDockAlignNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_robot()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()


# ros2 topic pub -1 /agv_dock_align std_msgs/msg/String "{data: 'tagStandard52h13:108'}"
