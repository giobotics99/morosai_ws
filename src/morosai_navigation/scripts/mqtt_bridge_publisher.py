#!/usr/bin/env python3
"""
MQTT Bridge Publisher Node

This node bridges standard NAV2 topics to custom morosai_navigation messages
that are automatically converted to/from JSON by the mqtt_bridge_node.

The mqtt_bridge_node handles ROS2 <-> MQTT conversion transparently:
  - ROS2 messages are serialized to JSON with identical field names
  - MQTT JSON messages are deserialized back to ROS2 messages
  - Same topic name is used on both ROS2 and MQTT sides

Published Topics (custom msgs -> MQTT via bridge):
    /agv_pose       morosai_navigation/Pose       <- /amcl_pose
    /agv_path       morosai_navigation/Path       <- /plan
    /agv_detect     morosai_navigation/Detect     <- /global_costmap/costmap + /yolo_detections/results
    /agv_qr         optical_head/PgvScanData      <- /pgv100_scan (remap)
    /nav_status     morosai_navigation/NavStatus   (periodic state machine)

Additional Subscriptions (perception):
    /yolo_detections/results  std_msgs/String      <- yolo_tof_detector node (JSON)
    /gordon_tof/pointcloud    sensor_msgs/PointCloud2 <- ToF camera (3-D lookup)

Subscribed Topics (MQTT via bridge -> ROS2):
    /agv_act        morosai_navigation/Pose       -> NavigateToPose action goal

Usage:
    ros2 run morosai_navigation mqtt_bridge_publisher.py
"""

import math
import struct
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rclpy.qos import qos_profile_sensor_data

import json
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped, Twist, PointStamped
from nav_msgs.msg import Path, OccupancyGrid
from sensor_msgs.msg import PointCloud2, CameraInfo
from std_msgs.msg import String
import tf2_ros

from tf2_geometry_msgs import do_transform_pose
from nav2_msgs.action import NavigateToPose
from morosai_navigation.msg import Pose as AgvPose
from morosai_navigation.msg import Path as AgvPath
from morosai_navigation.msg import Detect as AgvDetect
from morosai_navigation.msg import NavStatus
from morosai_navigation.msg import LocalPath as AgvLocalPath
from morosai_navigation.msg import Tag as AgvTag
from morosai_navigation.msg import FinalPath as AgvFinalPath

# Optical head message (remapped as-is)
try:
    from optical_head.msg import PgvScanData
    HAS_OPTICAL_HEAD = True
    print("TRUE CASE HAS_OPTICAL_HEAD")
except ImportError:
    HAS_OPTICAL_HEAD = False
    print("[WARN] optical_head package not found, /agv_qr remap will not be active")

# AprilTag message (standard christianrauch/apriltag_ros)
try:
    from apriltag_msgs.msg import AprilTagDetectionArray
    HAS_APRILTAG = True
except ImportError:
    HAS_APRILTAG = False
    print("[WARN] apriltag_msgs package not found, /agv_tag will not be active")


class MqttBridgePublisher(Node):
    """
    ROS2 Node that bridges NAV2 topics to custom morosai messages
    for automatic MQTT conversion by the mqtt_bridge_node.
    """

    def __init__(self):
        super().__init__('mqtt_bridge_publisher')

        # ── QoS Profiles ─────────────────────────────────────────────
        qos_reliable = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )

        qos_transient = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=1
        )

        # ── Internal State ────────────────────────────────────────────
        self._latest_pose_x = 0.0
        self._latest_pose_y = 0.0
        self._latest_pose_yaw = 0.0
        self._latest_pose_frame = 'map'
        self._latest_cmd_vel = 0.0
        self._has_pose = False
        
        self._agv_operation = {"status": "IDLE", "command": None}
        self._current_goals = []
        
        self._goal_handle = None
        self._last_agv_act_goal = {"x": None, "y": None, "angle": None}
        
        # Path history for FINAL path
        self._actual_path_history = []
        self._last_path_record_time = 0.0

        # Enriched Nav Status State
        self._distance_remaining = 0.0
        self._recoveries_count = 0
        self._nav_info = "IDLE"
        self._latest_cmd_vel_linear = 0.0
        self._has_plan = False
        self._last_inter_pub_time = 0.0   # throttle INTER on /agv_path to 1 Hz



        # —— TF2 Listener (for corrected deviation) ─────────────────────
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Subscriptions
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self._amcl_pose_cb, qos_reliable)
        # Nav2 global planner publishes /plan as TRANSIENT_LOCAL — must use
        # matching QoS or ROS2 silently drops the subscription (VOLATILE is
        # incompatible with TRANSIENT_LOCAL and yields zero messages).
        self.create_subscription(Path, '/plan', self._plan_cb, 10)
        self.create_subscription(AgvPose, '/agv_act', self._agv_act_cb, qos_reliable)
        # Intercept RViz2 goals so they update nav_status just like /agv_act
        self.create_subscription(PoseStamped, '/goal_pose', self._rviz_goal_cb, qos_reliable)
        
        # self.create_subscription(String, '/agv_op_CMD', self._agv_op_cmd_cb, qos_reliable)
        self.create_subscription(Twist, '/cmd_vel', self._cmd_vel_cb, qos_reliable)

        # Publishers
        self.agv_pose_pub = self.create_publisher(AgvPose, '/agv_pose', 10)
        self.agv_path_pub = self.create_publisher(AgvPath, '/agv_path', 10)
        self.nav_status_pub = self.create_publisher(NavStatus, '/nav_status', 10)

        # Action Client
        self._nav_action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # Subscriptions
        self.create_subscription(OccupancyGrid, '/global_costmap/costmap', self._costmap_cb, qos_transient)
        self.create_subscription(Path, '/local_plan', self._local_plan_cb, qos_reliable)
        self.create_subscription(Path, '/transformed_global_plan', self._transformed_global_plan_cb, qos_reliable)



        if HAS_OPTICAL_HEAD:
            self.create_subscription(PgvScanData, '/optical_head/pgv100_scan', self._pgv_remap_cb, qos_reliable)

        if HAS_APRILTAG:
            self.create_subscription(AprilTagDetectionArray, '/detections', self._apriltag_cb, qos_reliable)

        # Publishers
        self.agv_detect_pub = self.create_publisher(AgvDetect, '/agv_detect', 10)
        self.agv_local_path_pub = self.create_publisher(AgvLocalPath, '/agv_local_path', 10)
        
        if HAS_OPTICAL_HEAD:
            self.agv_qr_pub = self.create_publisher(PgvScanData, '/agv_qr', 10)

        self.agv_tag_pub = self.create_publisher(AgvTag, '/agv_tag', 10)
        self.agv_final_path_pub = self.create_publisher(AgvFinalPath, '/agv_final_path', 10)

        # ── Periodic nav_status publisher (1 Hz) ─────────────────────
        self.create_timer(1.0, self._publish_nav_status)

        self.get_logger().info('MqttBridgePublisher initialized')
        self.get_logger().info(
            'Publishing: /agv_pose, /agv_path, /agv_local_path, /agv_detect, /agv_qr, /agv_tag, /nav_status'
        )
        self.get_logger().info(
            'Subscribing: /agv_act (goal from scheduler)'
        )

    def _amcl_pose_cb(self, msg: PoseWithCovarianceStamped):
        """
        /amcl_pose → /agv_pose
        Extract x, y, yaw from PoseWithCovarianceStamped and publish
        as morosai_navigation/Pose.
        """
        pose = msg.pose.pose
        yaw = self._quaternion_to_yaw(pose.orientation)

        # Cache for use in costmap and deviation calculation
        self._latest_pose_x = pose.position.x
        self._latest_pose_y = pose.position.y
        self._latest_pose_yaw = float(yaw)
        self._latest_pose_frame = msg.header.frame_id
        self._has_pose = True

        agv_pose = AgvPose()
        agv_pose.x = float(pose.position.x)
        agv_pose.y = float(pose.position.y)
        agv_pose.angle = float(yaw)

        # Record history for FINAL path if navigating
        if self._agv_operation["status"] in ["NAVIGATING", "MOVING", "PLANNING", "RECOVERING"]:
            current_time = self.get_clock().now().nanoseconds / 1e9
            if current_time - self._last_path_record_time >= 1.0:
                p = AgvPose()
                p.x = float(pose.position.x)
                p.y = float(pose.position.y)
                p.angle = float(yaw)
                self._actual_path_history.append(p)
                self._last_path_record_time = current_time

        self.agv_pose_pub.publish(agv_pose)

    def _plan_cb(self, msg: Path):
        """
        /plan → /agv_path
        Convert nav_msgs/Path to morosai_navigation/Path.
        Each PoseStamped in the plan becomes a morosai_navigation/Pose.
        """
        agv_path = AgvPath()
        agv_path.type = "INIT"

        for pose_stamped in msg.poses:
            p = AgvPose()
            p.x = float(pose_stamped.pose.position.x)
            p.y = float(pose_stamped.pose.position.y)
            p.angle = float(
                self._quaternion_to_yaw(pose_stamped.pose.orientation)
            )
            agv_path.path.append(p)

        self._has_plan = len(agv_path.path) > 0
        # Reset throttle so INTER can follow INIT immediately after a new goal
        self._last_inter_pub_time = 0.0
        self.agv_path_pub.publish(agv_path)
        self.get_logger().info(
            f'[PLAN] /agv_path INIT published with {len(agv_path.path)} waypoints'
        )

    def _local_plan_cb(self, msg: Path):
        """
        /local_plan → /agv_local_path
        Republishes the Nav2 local plan as a custom morosai LocalPath message.
        """
        if not msg.poses:
            return

        plan_frame = msg.header.frame_id

        # Populate LocalPath message
        local_path = AgvLocalPath()
        local_path.frame_id = str(plan_frame)

        for pose_stamped in msg.poses:
            p = AgvPose()
            p.x = float(pose_stamped.pose.position.x)
            p.y = float(pose_stamped.pose.position.y)
            p.angle = float(self._quaternion_to_yaw(pose_stamped.pose.orientation))
            local_path.path.append(p)

        self.agv_local_path_pub.publish(local_path)
        
    def _transformed_global_plan_cb(self, msg: Path):
        """
        /transformed_global_plan → /agv_path (INTER)
        
        Nav2's controller extracts a chunk of the global plan up to the
        lookahead distance as the `transformed_global_plan`. We use this
        as the INTER path and transform it to 'map' frame if necessary.
        """
        if not self._has_plan:
            return  # INIT not yet published — don't race with it

        now = self.get_clock().now().nanoseconds / 1e9
        if (now - self._last_inter_pub_time) < 1.0:   # 1 Hz cap
            return

        if not msg.poses:
            return

        source_frame = msg.header.frame_id
        target_frame = 'map'
        needs_transform = (source_frame != target_frame)
        transform = None

        if needs_transform:
            try:
                transform = self.tf_buffer.lookup_transform(
                    target_frame, source_frame, rclpy.time.Time()
                )
            except Exception as exc:
                self.get_logger().debug(f"Could not transform {source_frame} to {target_frame}: {exc}")
                return

        inter_path = AgvPath()
        inter_path.type = "INTER"

        for pose_stamped in msg.poses:
            if needs_transform:
                tf_pose = do_transform_pose(pose_stamped.pose, transform)
                p = AgvPose()
                p.x = float(tf_pose.position.x)
                p.y = float(tf_pose.position.y)
                p.angle = float(self._quaternion_to_yaw(tf_pose.orientation))
            else:
                p = AgvPose()
                p.x = float(pose_stamped.pose.position.x)
                p.y = float(pose_stamped.pose.position.y)
                p.angle = float(self._quaternion_to_yaw(pose_stamped.pose.orientation))
            
            inter_path.path.append(p)

        self.agv_path_pub.publish(inter_path)
        self._last_inter_pub_time = now
        self.get_logger().debug(
            f'/agv_path INTER published with {len(inter_path.path)} points'
        )



    def _costmap_cb(self, msg: OccupancyGrid):
        """
        /global_costmap/costmap → /agv_detect

        Publish type=OBSTACLE (or CLEAR) based on standard costmap lethal-cell scan.
        """
        detect = AgvDetect()

        if not self._has_pose:
            detect.type = "CLEAR"
            detect.pose = AgvPose(x=0.0, y=0.0, angle=0.0)
            self.agv_detect_pub.publish(detect)
            return


        radius = 2.0  # metres
        resolution = msg.info.resolution
        width      = msg.info.width
        height     = msg.info.height
        origin_x   = msg.info.origin.position.x
        origin_y   = msg.info.origin.position.y

        col_robot    = int((self._latest_pose_x - origin_x) / resolution)
        row_robot    = int((self._latest_pose_y - origin_y) / resolution)
        cells_radius = int(radius / resolution)

        col_start = max(0, col_robot - cells_radius)
        col_end   = min(width,  col_robot + cells_radius)
        row_start = max(0, row_robot - cells_radius)
        row_end   = min(height, row_robot + cells_radius)

        min_dist_sq    = float('inf')
        nearest_x      = 0.0
        nearest_y      = 0.0
        found_obstacle = False

        for row in range(row_start, row_end):
            for col in range(col_start, col_end):
                idx = row * width + col
                if 0 <= idx < len(msg.data) and msg.data[idx] >= 100:
                    cell_x = origin_x + (col + 0.5) * resolution
                    cell_y = origin_y + (row + 0.5) * resolution
                    dx = cell_x - self._latest_pose_x
                    dy = cell_y - self._latest_pose_y
                    dist_sq = dx * dx + dy * dy
                    if dist_sq < min_dist_sq:
                        min_dist_sq    = dist_sq
                        nearest_x      = cell_x
                        nearest_y      = cell_y
                        found_obstacle = True

        if found_obstacle:
            dist          = math.sqrt(min_dist_sq)
            abs_angle     = math.atan2(
                nearest_y - self._latest_pose_y,
                nearest_x - self._latest_pose_x
            )
            angle_to_obs  = (abs_angle - self._latest_pose_yaw + math.pi) % (2 * math.pi) - math.pi
            detect.type     = "OBSTACLE"
            detect.distance = float(dist)
            detect.angle    = float(angle_to_obs)
            detect.pose     = AgvPose(
                x=float(nearest_x),
                y=float(nearest_y),
                angle=float(angle_to_obs),
            )
        else:
            detect.type          = "CLEAR"
            detect.distance      = 0.0
            detect.angle         = 0.0
            detect.pose          = AgvPose(x=0.0, y=0.0, angle=0.0)

        detect.volume_occupancy = self._compute_lethal_area(msg)
        self.agv_detect_pub.publish(detect)

    def _compute_lethal_area(self, msg: OccupancyGrid) -> float:
        """Return area (m²) of lethal cells in the 2 m window around the robot."""
        resolution = msg.info.resolution
        width      = msg.info.width
        height     = msg.info.height
        origin_x   = msg.info.origin.position.x
        origin_y   = msg.info.origin.position.y
        radius     = 2.0

        col_robot    = int((self._latest_pose_x - origin_x) / resolution)
        row_robot    = int((self._latest_pose_y - origin_y) / resolution)
        cells_radius = int(radius / resolution)

        col_start = max(0, col_robot - cells_radius)
        col_end   = min(width,  col_robot + cells_radius)
        row_start = max(0, row_robot - cells_radius)
        row_end   = min(height, row_robot + cells_radius)

        lethal_count = 0
        for row in range(row_start, row_end):
            for col in range(col_start, col_end):
                idx = row * width + col
                if 0 <= idx < len(msg.data) and msg.data[idx] >= 100:
                    lethal_count += 1

        return float(lethal_count * (resolution ** 2))

    def _rviz_goal_cb(self, msg: PoseStamped):
        """
        /goal_pose ← RViz2 "2D Goal Pose"
        Converts a standard ROS 2 RViz goal into our custom AgvPose format
        and feeds it into _agv_act_cb so that our MQTT state machine (nav_status)
        correctly monitors and reports it as if it came from the Scheduler!
        """
        pose_to_use = AgvPose()
        pose_to_use.x = float(msg.pose.position.x)
        pose_to_use.y = float(msg.pose.position.y)
        pose_to_use.angle = float(self._quaternion_to_yaw(msg.pose.orientation))
        
        self.get_logger().info('Intercepted manual goal from RViz2 (/goal_pose)')
        self._agv_act_cb(pose_to_use)

    def _agv_act_cb(self, msg):
        """
        /agv_act ← Scheduler (via MQTT bridge)
        Receives a goal pose (or array of poses) from the external scheduler and sends it
        as a NavigateToPose action goal to the Nav2 stack if it has changed.
        """
        # Handle array of poses or single pose
        if isinstance(msg, list):
            if not msg:
                return
            pose_to_use = msg[0]
        else:
            pose_to_use = msg

        # Check if the goal has changed
        if (self._last_agv_act_goal["x"] == pose_to_use.x and
            self._last_agv_act_goal["y"] == pose_to_use.y and
            self._last_agv_act_goal["angle"] == pose_to_use.angle):
            return

        self.get_logger().info(
            f'/agv_act received goal change: x={pose_to_use.x:.2f}, y={pose_to_use.y:.2f}, '
            f'angle={pose_to_use.angle:.2f}'
        )

        # Update last goal
        self._last_agv_act_goal["x"] = pose_to_use.x
        self._last_agv_act_goal["y"] = pose_to_use.y
        self._last_agv_act_goal["angle"] = pose_to_use.angle

        # Wait for the action server (with timeout)
        if not self._nav_action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error(
                'NavigateToPose action server not available!'
            )
            self._agv_operation["status"] = "ERROR"
            self._agv_operation["command"] = "GOTO"
            return

        # Build the NavigateToPose goal
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = PoseStamped()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = float(pose_to_use.x)
        goal_msg.pose.pose.position.y = float(pose_to_use.y)
        goal_msg.pose.pose.position.z = 0.0

        # Convert yaw angle back to quaternion
        qz, qw = self._yaw_to_quaternion(pose_to_use.angle)
        goal_msg.pose.pose.orientation.x = 0.0
        goal_msg.pose.pose.orientation.y = 0.0
        goal_msg.pose.pose.orientation.z = qz
        goal_msg.pose.pose.orientation.w = qw

        # Send goal asynchronously
        self._agv_operation["status"] = "PLANNING"
        self._agv_operation["command"] = "GOTO"
        self._nav_info = "Goal sent, calculating plan..."
        self._current_goals = [goal_msg]
        self._has_plan = False
        self._distance_remaining = 0.0
        self._recoveries_count = 0

        send_goal_future = self._nav_action_client.send_goal_async(
            goal_msg,
            feedback_callback=self._nav_feedback_cb
        )
        send_goal_future.add_done_callback(self._nav_goal_response_cb)

        # Clear path history for new navigation and record starting pose
        self._actual_path_history = []
        self._last_path_record_time = self.get_clock().now().nanoseconds / 1e9
        
        # Add initial pose if available
        if self._has_pose:
            start_pose = AgvPose()
            start_pose.x = float(self._latest_pose_x)
            start_pose.y = float(self._latest_pose_y)
            start_pose.angle = float(self._latest_pose_yaw)
            self._actual_path_history.append(start_pose)

        self.get_logger().info('NavigateToPose goal sent to Nav2')

        self.get_logger().info('NavigateToPose goal sent to Nav2')

    # def _agv_op_cmd_cb(self, msg: String):
    #     """Handle incoming operator commands (/agv_op_CMD)"""
    #     try:
    #         cmd = json.loads(msg.data)
    #         self._agv_operation["command"] = cmd.get("command", None)
    #         self._agv_operation["status"] = cmd.get("status", self._agv_operation["status"])
            
    #         # Handle specific commands
    #         if cmd.get("command") == "STOP":
    #             self._current_goals = []
    #             self._agv_operation["status"] = "STOPPED"
    #             if self._goal_handle:
    #                 self._goal_handle.cancel_goal_async()
    #         elif cmd.get("command") == "PAUSE":
    #             self._agv_operation["status"] = "PAUSED"
    #         elif cmd.get("command") == "RESUME":
    #             self._agv_operation["status"] = "NAVIGATING"
    #         elif cmd.get("command") == "CLEAR_GOALS":
    #             self._current_goals = []
    #             self._agv_operation["status"] = "IDLE"
    #             if self._goal_handle:
    #                 self._goal_handle.cancel_goal_async()
                
    #         # If waypoints are provided, update goals
    #         if "waypoints" in cmd:
    #             self._current_goals = cmd["waypoints"]
    #             self._agv_operation["status"] = "NAVIGATING"
                
    #     except json.JSONDecodeError:
    #       self.get_logger().warn(f"Invalid JSON in agv_op_CMD: {msg.data}")

    def _cmd_vel_cb(self, msg: Twist):
        """Monitor robot velocity to detect actual movement."""
        self._latest_cmd_vel_linear = abs(msg.linear.x) + abs(msg.linear.y)

    def _pgv_remap_cb(self, msg):
        """
        /pgv100_scan → /agv_qr
        Simple topic remap: republish the PgvScanData message as-is
        on /agv_qr. The mqtt_bridge handles JSON conversion.
        """
        self.agv_qr_pub.publish(msg)

    def _apriltag_cb(self, msg: AprilTagDetectionArray):
        """
        /detections → /agv_tag
        Converts AprilTag detections to custom morosai Tag messages.
        """
        camera_frame = msg.header.frame_id
        for detection in msg.detections:
            tag = AgvTag()
            tag_id = getattr(detection, 'id', 0)
            fam = getattr(detection, 'family', 'tagStandard52h13')
            tag.id = int(tag_id)
            
            # The standard frame name for christianrauch/apriltag_ros is usually '{family}:{id}'
            tag_frame = f"{fam}:{tag_id}"
            
            try:
                # Try to look up transform from 'map' to the tag frame first
                # If the robot is not localized, this might fail, so we fallback to camera_frame
                try:
                    target_frame = 'map'
                    trans = self.tf_buffer.lookup_transform(target_frame, tag_frame, rclpy.time.Time())
                except Exception:
                    # Fallback to the camera frame if map is unavailable
                    target_frame = camera_frame
                    trans = self.tf_buffer.lookup_transform(target_frame, tag_frame, rclpy.time.Time())
                
                tag.x = float(trans.transform.translation.x)
                tag.y = float(trans.transform.translation.y)
                tag.z = float(trans.transform.translation.z)
                
                roll, pitch, yaw = self._quaternion_to_euler(trans.transform.rotation)
                tag.roll = float(roll)
                tag.pitch = float(pitch)
                tag.yaw = float(yaw)
                
                self.agv_tag_pub.publish(tag)
                self.get_logger().info(f'AprilTag {tag.id} reported via TF lookup in {target_frame}: {tag.x:.2f}, {tag.y:.2f}, {tag.z:.2f}')
                
            except Exception as exc:
                self.get_logger().warn(f"TF lookup for {tag_frame} failed completely: {exc}")

    def _nav_goal_response_cb(self, future):
        """Called when Nav2 accepts or rejects the goal."""
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().warn('NavigateToPose goal was REJECTED')
            self._agv_operation["status"] = "ERROR"
            self._current_goals = []
            return

        self.get_logger().info('NavigateToPose goal ACCEPTED')
        self._goal_handle = goal_handle

        # Wait for the result
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._nav_result_cb)

    def _nav_feedback_cb(self, feedback_msg):
        """Called periodically with navigation progress feedback."""
        feedback = feedback_msg.feedback
        self._distance_remaining = float(feedback.distance_remaining)
        
        # Check for recovery increase
        if feedback.number_of_recoveries > self._recoveries_count:
            self._agv_operation["status"] = "RECOVERING"
            self._nav_info = f"Recovery behavior active (#{feedback.number_of_recoveries})"
        
        self._recoveries_count = feedback.number_of_recoveries
        
        # Determine internal status based on plan availability and movement
        # If we are in RECOVERING or PAUSED/STOPPED, we don't override it here
        if self._agv_operation["status"] not in ["RECOVERING", "PAUSED", "STOPPED", "ERROR"]:
            if self._latest_cmd_vel_linear > 0.05:
                self._agv_operation["status"] = "MOVING"
                self._nav_info = "Directing to goal"
            elif self._has_plan:
                self._agv_operation["status"] = "NAVIGATING"
                self._nav_info = "Plan active, maneuvering"
            else:
                self._agv_operation["status"] = "PLANNING"
                self._nav_info = "Calculating path..."

    def _nav_result_cb(self, future):
        """Called when navigation completes (success or failure)."""
        result = future.result()
        status = result.status

        if status == 4:  # SUCCEEDED
            self.get_logger().info('Navigation goal SUCCEEDED')
            self._agv_operation["status"] = "IDLE"
            self._agv_operation["command"] = ""
            self._nav_info = "Goal reached successfully"
            self._distance_remaining = 0.0
            self._has_plan = False
            
            # Publish Final Paths
            if self._actual_path_history:
                # 1. Add the final exact pose reached to the history list first
                end_pose = AgvPose()
                end_pose.x = float(self._latest_pose_x)
                end_pose.y = float(self._latest_pose_y)
                end_pose.angle = float(self._latest_pose_yaw)
                self._actual_path_history.append(end_pose)

                # 2. Publish to /agv_path (type="FINAL") for UI visualization
                final_path = AgvPath()
                final_path.type = "FINAL"
                final_path.path = self._actual_path_history
                self.agv_path_pub.publish(final_path)
                self.get_logger().info(f'/agv_path FINAL published with {len(self._actual_path_history)} points')

                # 3. Publish to dedicated /agv_final_path for Mission Reports
                final_msg = AgvFinalPath()
                final_msg.path = self._actual_path_history
                self.agv_final_path_pub.publish(final_msg)
                self.get_logger().info(f'/agv_final_path published with {len(self._actual_path_history)} points')
        elif status == 5:  # CANCELED
            self.get_logger().warn('Navigation goal CANCELED')
            self._agv_operation["status"] = "STOPPED"
            self._agv_operation["command"] = ""
        elif status == 6:  # ABORTED
            self.get_logger().error('Navigation goal ABORTED')
            self._agv_operation["status"] = "ERROR"
            self._agv_operation["command"] = ""
            self._nav_info = "Navigation aborted by stack"
        else:
            self.get_logger().warn(f'Navigation ended with status: {status}')
            self._agv_operation["status"] = "ERROR"
            self._agv_operation["command"] = ""

        self._current_goals = []
        self._goal_handle = None

    def _publish_nav_status(self):
        """
        Publish /nav_status at 1 Hz — navigation state machine.
        """
        status_msg = NavStatus()
        status_msg.status = str(self._agv_operation["status"])
        status_msg.command = str(self._agv_operation["command"]) if self._agv_operation["command"] is not None else ""
        status_msg.is_navigating = (self._agv_operation["status"] in ["NAVIGATING", "MOVING", "PLANNING", "RECOVERING"])
        status_msg.is_idle = (self._agv_operation["status"] == "IDLE")
        status_msg.has_goals = len(self._current_goals) > 0
        
        # Enriched fields
        status_msg.distance_remaining = float(self._distance_remaining)
        status_msg.recoveries = int(self._recoveries_count)
        status_msg.info = str(self._nav_info)

        self.nav_status_pub.publish(status_msg)

    # ══════════════════════════════════════════════════════════════════

    @staticmethod
    def _quaternion_to_euler(q):
        """
        Convert a quaternion to euler angles (roll, pitch, yaw) in radians.
        """
        # roll (x-axis rotation)
        sinr_cosp = 2 * (q.w * q.x + q.y * q.z)
        cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y)
        roll = math.atan2(sinr_cosp, cosr_cosp)

        # pitch (y-axis rotation)
        sinp = 2 * (q.w * q.y - q.z * q.x)
        if abs(sinp) >= 1:
            pitch = math.copysign(math.pi / 2, sinp) # use 90 degrees if out of range
        else:
            pitch = math.asin(sinp)

        # yaw (z-axis rotation)
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        return roll, pitch, yaw

    @staticmethod
    def _quaternion_to_yaw(orientation) -> float:
        """Convert quaternion to yaw angle (radians)."""
        siny_cosp = 2.0 * (orientation.w * orientation.z +
                           orientation.x * orientation.y)
        cosy_cosp = 1.0 - 2.0 * (orientation.y ** 2 + orientation.z ** 2)
        return math.atan2(siny_cosp, cosy_cosp)

    @staticmethod
    def _yaw_to_quaternion(yaw: float):
        """Convert yaw angle (radians) to quaternion (z, w) components."""
        qz = math.sin(yaw / 2.0)
        qw = math.cos(yaw / 2.0)
        return qz, qw


def main(args=None):
    rclpy.init(args=args)
    node = MqttBridgePublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()