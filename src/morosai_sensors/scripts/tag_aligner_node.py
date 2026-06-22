#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from tf2_ros import Buffer, TransformListener
import tf2_ros
import math
import numpy as np
import time  # <--- Importato per gestire il tempo in secondi reali

def quat_to_rot_matrix(q):
    """Convert a quaternion [x, y, z, w] to a 3x3 rotation matrix."""
    qx, qy, qz, qw = q
    return np.array([
        [1 - 2*qy**2 - 2*qz**2,     2*qx*qy - 2*qz*qw,     2*qx*qz + 2*qy*qw],
        [    2*qx*qy + 2*qz*qw, 1 - 2*qx**2 - 2*qz**2,     2*qy*qz - 2*qx*qw],
        [    2*qx*qz - 2*qy*qw,     2*qy*qz + 2*qx*qw, 1 - 2*qx**2 - 2*qy**2]
    ])

class TagAlignerNode(Node):
    def __init__(self):
        super().__init__('tag_aligner_node')
        
        # Parameters
        self.declare_parameter('target_tag_frame', 'tagStandard52h13:108')
        self.declare_parameter('camera_frame', 'tof_optical_frame')
        self.declare_parameter('robot_frame', 'base_footprint')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel_raw')
        
        self.declare_parameter('target_distance', 1.5)
        
        self.declare_parameter('kp_x', 0.8)
        self.declare_parameter('kp_y', 0.8)
        self.declare_parameter('kp_yaw', 1.2)
        
        self.declare_parameter('max_linear_vel', 0.1)
        self.declare_parameter('max_angular_vel', 0.05)
        
        self.declare_parameter('xy_tolerance', 0.05)
        self.declare_parameter('yaw_tolerance', 0.05)
        
        self.target_tag_frame = self.get_parameter('target_tag_frame').value
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
        
        # --- LOGICA DI SICUREZZA E PAUSA ---
        self.is_paused = False
        self.pause_start_time = 0.0
        self.safety_wait_duration = 1.0  # Tempo di attesa obbligatorio (1 secondo)
        self.last_sent_linear_x = 0.0
        
        # TF2 Setup
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Publishers & Subscribers
        self.cmd_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        
        # Sottoscrizione all'output reale dei motori per intercettare gli stop di Nav2
        self.safety_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.safety_feedback_callback,
            10
        )
        
        # Control Loop
        self.timer = self.create_timer(0.1, self.control_loop)
        
        self.get_logger().info(f"Tag Aligner Node initialized.")
        self.get_logger().info(f"Aligning with {self.target_tag_frame} at {self.target_distance}m from {self.camera_frame}")

    def safety_feedback_callback(self, msg):
        """Monitora se il collision_monitor ha forzato uno STOP strutturale."""
        # Se noi stiamo chiedendo di muoverci ma l'output effettivo è zero,
        # significa che il collision monitor è intervenuto attivamente
        current_time = time.time()
        if abs(self.last_sent_linear_x) > 0.01 and abs(msg.linear.x) < 0.001:
            if not self.is_paused:
                self.is_paused = True
                self.pause_start_time = current_time
                self.get_logger().error("COLLISION MONITOR INTERVENUTO! Robot congelato per 1 secondo.")

    def control_loop(self):
        current_time = time.time()
        
        # --- Controllo dello stato di Pausa Obbligatoria ---
        if self.is_paused:
            if (current_time - self.pause_start_time) < self.safety_wait_duration:
                # Il secondo non è ancora passato, forza lo stop assoluto sul bus raw
                self.get_logger().warn("PAUSA DI SICUREZZA ATTIVA: Attesa ripristino...", throttle_duration_sec=0.5)
                self.stop_robot()
                return
            else:
                # Il secondo è scaduto, sblocca la macchina a stati
                self.is_paused = False
                self.get_logger().info("Pausa di sicurezza terminata. Riprendo il tracking del tag.")

        try:
            # Get transform from camera to tag
            t_cam_tag = self.tf_buffer.lookup_transform(
                self.camera_frame,
                self.target_tag_frame,
                rclpy.time.Time())
                
            # Get transform from base to camera
            t_base_cam = self.tf_buffer.lookup_transform(
                self.robot_frame,
                self.camera_frame,
                rclpy.time.Time())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            self.stop_robot()
            return
            
        # --- 1. Compute Errors in Camera Frame ---
        x = t_cam_tag.transform.translation.x
        z = t_cam_tag.transform.translation.z
        
        error_lat = x           
        error_fwd = z - self.target_distance  
        
        q_tag = [
            t_cam_tag.transform.rotation.x,
            t_cam_tag.transform.rotation.y,
            t_cam_tag.transform.rotation.z,
            t_cam_tag.transform.rotation.w
        ]
        R_cam_tag = quat_to_rot_matrix(q_tag)
        tag_z_axis = R_cam_tag @ np.array([0, 0, 1])
        
        if tag_z_axis[2] < 0:
            tag_z_axis = -tag_z_axis
            
        theta = math.atan2(tag_z_axis[0], tag_z_axis[2])
        error_yaw = theta
        
        # --- 2. Compute Desired Velocities in Camera Frame ---
        v_cam_x = 0.0
        v_cam_z = 0.0
        w_cam_y = 0.0
        
        if abs(error_lat) > self.xy_tol:
            v_cam_x = self.kp_x * error_lat
            
        if abs(error_fwd) > self.xy_tol:
            v_cam_z = self.kp_y * error_fwd
            
        if abs(error_yaw) > self.yaw_tol:
            w_cam_y = self.kp_yaw * error_yaw
            
        # Clamp camera velocities
        v_cam_x = max(-self.max_lin_vel, min(self.max_lin_vel, v_cam_x))
        v_cam_z = max(-self.max_lin_vel, min(self.max_lin_vel, v_cam_z))
        w_cam_y = max(-self.max_ang_vel, min(self.max_ang_vel, w_cam_y))
        
        # --- 3. Transform Velocities to Robot Base Frame ---
        q_base_cam = [
            t_base_cam.transform.rotation.x,
            t_base_cam.transform.rotation.y,
            t_base_cam.transform.rotation.z,
            t_base_cam.transform.rotation.w
        ]
        R_base_cam = quat_to_rot_matrix(q_base_cam)
        
        V_cam = np.array([v_cam_x, 0.0, v_cam_z])
        W_cam = np.array([0.0, w_cam_y, 0.0])
        
        V_base = R_base_cam @ V_cam
        W_base = R_base_cam @ W_cam
        
        # --- 4. Publish Command ---
        twist = Twist()
        twist.linear.x = float(V_base[0])
        twist.linear.y = float(V_base[1])
        twist.angular.z = float(W_base[2])

        # Memorizza l'ultimo comando inviato per usarlo nel confronto della sicurezza
        self.last_sent_linear_x = twist.linear.x

        self.get_logger().info(f"PUBBLICO SU RAW -> x: {twist.linear.x:.2f}, yaw: {twist.angular.z:.2f}")
        self.cmd_pub.publish(twist)

    def stop_robot(self):
        twist = Twist()
        self.last_sent_linear_x = 0.0
        self.cmd_pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = TagAlignerNode()
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