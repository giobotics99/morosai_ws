#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String  # Usiamo stringhe standard per facilitare il bridge MQTT
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

class TagAlignNode(Node):
    def __init__(self):
        super().__init__('tag_align_node')
        
        # --- PARAMETRI ---
        self.declare_parameter('allowed_tags', ['tagStandard52h13:104', 'tagStandard52h13:103', 'tagStandard52h13:102'])
        self.declare_parameter('camera_frame', 'tof_optical_frame')
        self.declare_parameter('robot_frame', 'base_footprint')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel_raw')
        self.declare_parameter('target_distance', 0.5)
        self.declare_parameter('kp_x', 0.8)
        self.declare_parameter('kp_y', 0.8)
        self.declare_parameter('kp_yaw', 1.2)
        self.declare_parameter('max_linear_vel', 0.1)
        self.declare_parameter('max_angular_vel', 0.05)
        self.declare_parameter('xy_tolerance', 0.05)
        self.declare_parameter('yaw_tolerance', 0.05)
        self.declare_parameter('alignment_timeout', 15.0)
        
        self.allowed_tags = self.get_parameter('allowed_tags').value
        self.camera_frame = self.get_parameter('camera_frame').value
        self.robot_frame = self.get_parameter('robot_frame').value
        self.target_distance = self.get_parameter('target_distance').value
        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        self.kp_x = self.get_parameter('kp_x').value
        self.kp_y = self.get_parameter('kp_y').value
        self.kp_yaw = self.get_parameter('kp_yaw').value
        self.max_lin_vel = self.get_parameter('max_linear_vel').value
        self.max_ang_vel = self.get_parameter('max_angular_vel').value
        self.xy_tol = self.get_parameter('xy_tolerance').value
        self.yaw_tol = self.get_parameter('yaw_tolerance').value
        self.alignment_timeout = self.get_parameter('alignment_timeout').value
        
        # --- STATO INTERNO CORRELAZIONE EVENTI ---
        self.target_tag_frame = ""
        self.active_alignment = False   # Stato IDLE di default
        self.start_time = None
        
        # Sicurezza collisioni
        self.is_paused = False
        self.pause_start_time = 0.0
        self.safety_wait_duration = 1.0  
        self.last_sent_linear_x = 0.0
        
        # TF2 Setup
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # --- PUB/SUB ROS 2 (Pronti per MQTT Bridge) ---
        self.cmd_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.result_pub = self.create_publisher(String, '/align_result', 10)
        
        self.command_sub = self.create_subscription(String, '/agv_align', self.command_callback, 10)
        self.safety_sub = self.create_subscription(Twist, '/cmd_vel', self.safety_feedback_callback, 10)
        
        # Il loop di controllo gira a 10Hz, ma non esegue calcoli se active_alignment == False
        self.timer = self.create_timer(0.1, self.control_loop)
        
        self.get_logger().info("Nodo Tag Aligner basato su TOPIC avviato in IDLE.")

    def safety_feedback_callback(self, msg):
        current_time = time.time()
        if abs(self.last_sent_linear_x) > 0.01 and abs(msg.linear.x) < 0.001:
            if not self.is_paused:
                self.is_paused = True
                self.pause_start_time = current_time
                self.get_logger().error("COLLISION MONITOR INTERVENUTO!")

    def command_callback(self, msg):
        """Riceve il comando dallo scheduler tramite MQTT -> ROS2 Bridge."""
        requested_tag = msg.data
        self.get_logger().info(f"Ricevuto comando per tag: {requested_tag}")
        
        # Aggiorna al volo i parametri di fabbrica
        self.allowed_tags = self.get_parameter('allowed_tags').value
        
        # 1. Controllo Lista Tag Consentiti
        if requested_tag not in self.allowed_tags:
            self.get_logger().error(f"Tag {requested_tag} NON consentito.")
            self.publish_result("REJECTED_NOT_ALLOWED")
            return
            
        # 2. Controllo Visibilità Istantanea
        try:
            self.tf_buffer.lookup_transform(
                self.camera_frame, requested_tag, rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.1)
            )
            # Se visibile, attiva la macchina a stati
            self.target_tag_frame = requested_tag
            self.start_time = self.get_clock().now()
            self.active_alignment = True
            self.get_logger().info(f"Tag {requested_tag} agganciato. Avvio allineamento.")
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            self.get_logger().warn(f"Tag {requested_tag} consentito ma NON visibile. Comando ignorato.")
            self.publish_result("REJECTED_NOT_VISIBLE")

    def control_loop(self):
        # Clausola di guardia: se non siamo in modalità allineamento attivo, non consumare CPU
        if not self.active_alignment:
            return

        # 3. Controllo Timeout di Sicurezza (Gestito nel loop per non bloccare i thread)
        elapsed = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
        if elapsed > self.alignment_timeout:
            self.get_logger().error("TIMEOUT SCADUTO. Allineamento fallito.")
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
            self.stop_robot() # Se perdi il frame momentaneamente ti fermi, ma non spegni il servizio fino al timeout
            return
            
        # Calcolo Errori
        x = t_cam_tag.transform.translation.x
        z = t_cam_tag.transform.translation.z
        error_lat = x           
        error_fwd = z - self.target_distance  
        
        q_tag = [t_cam_tag.transform.rotation.x, t_cam_tag.transform.rotation.y, t_cam_tag.transform.rotation.z, t_cam_tag.transform.rotation.w]
        R_cam_tag = quat_to_rot_matrix(q_tag)
        tag_z_axis = R_cam_tag @ np.array([0, 0, 1])
        if tag_z_axis[2] < 0: tag_z_axis = -tag_z_axis
        theta = math.atan2(tag_z_axis[0], tag_z_axis[2])
        error_yaw = theta
        
        # 4. Condizione di Convergenza Ottimale
        if abs(error_lat) <= self.xy_tol and abs(error_fwd) <= self.xy_tol and abs(error_yaw) <= self.yaw_tol:
            self.get_logger().info("ALLINEAMENTO COMPLETATO CON SUCCESSO.")
            self.stop_robot()
            self.active_alignment = False # Ritorna in IDLE
            self.publish_result("SUCCESS")
            return
        
        # Calcolo velocità e invio ai motori
        v_cam_x = self.kp_x * error_lat if abs(error_lat) > self.xy_tol else 0.0
        v_cam_z = self.kp_y * error_fwd if abs(error_fwd) > self.xy_tol else 0.0
        w_cam_y = self.kp_yaw * error_yaw if abs(error_yaw) > self.yaw_tol else 0.0
        
        v_cam_x = max(-self.max_lin_vel, min(self.max_lin_vel, v_cam_x))
        v_cam_z = max(-self.max_lin_vel, min(self.max_lin_vel, v_cam_z))
        w_cam_y = max(-self.max_ang_vel, min(self.max_ang_vel, w_cam_y))
        
        q_base_cam = [t_base_cam.transform.rotation.x, t_base_cam.transform.rotation.y, t_base_cam.transform.rotation.z, t_base_cam.transform.rotation.w]
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
    node = TagAlignNode()
    try:
        rclpy.spin(node) # SingleThreadedExecutor standard: ora basta e avanza perché non ci sono while bloccanti!
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_robot()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()