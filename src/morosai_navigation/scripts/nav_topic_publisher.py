#!/usr/bin/env python3
"""
Navigation Topic Publisher Node

This node subscribes to standard NAV2 topics and republishes them
with custom /agv_* topic names for integration with external systems.

Topic Mappings:
    /amcl_pose         -> /agv_pose    (robot pose from localization)
    /plan              -> /agv_path    (planned navigation path)
    /local_plan        -> /agv_local_path (local path from DWB planner)
    Deviation          -> /agv_deviation  (min distance: robot -> local plan)
    /cmd_vel           -> /agv_v       (velocity commands)
    /pgv100_scan       -> /agv_qr      (optical head QR/tag detection)
    Costmap            -> /agv_detect  (obstacle detection info)
    /joint_states      -> /wheel_v     (wheel velocities)
    NavigateToPose     -> /agv_act     (navigation goals/waypoints)
    Commands in        -> /agv_op      (operator commands)


To view published topics and save them in a file txt use:
ros2 topic echo /agv_act > output.txt

Usage:
    ros2 run morosai_navigation nav_topic_publisher.py
"""

import json
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

# Standard message types
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist, TwistStamped, PoseStamped
from nav_msgs.msg import Path, OccupancyGrid
from std_msgs.msg import String, Float32
from sensor_msgs.msg import JointState

# NAV2 action for goal monitoring
from nav2_msgs.action import NavigateToPose, NavigateThroughPoses

# Custom message from optical_head package
try:
    from optical_head.msg import PgvScanData
    HAS_OPTICAL_HEAD = True
except ImportError:
    HAS_OPTICAL_HEAD = False
    print("[WARN] optical_head package not found, /agv_qr will not be published")


class NavTopicPublisher(Node):
    """
    ROS2 Node that republishes NAV2 topics to custom AGV topic names.
    """

    def __init__(self):
        super().__init__('nav_topic_publisher')
        
        # QoS profile for reliable communication
        qos_reliable = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )
        
        # QoS for transient local (for costmap/plan that may be latched)
        qos_transient = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=1
        )

        # State for agv_act (goal poses)
        self._current_goals = []
        self._agv_operation = {"status": "IDLE", "command": None}
        self._latest_pose = None  # Store latest /amcl_pose for deviation calc

        # ============================================================
        # SUBSCRIBERS - Standard NAV2 topics
        # ============================================================
        
        # /amcl_pose - Robot pose from localization
        self.pose_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self.pose_callback,
            qos_reliable
        )
        
        # /plan - Planned path from planner_server
        self.path_sub = self.create_subscription(
            Path,
            '/plan',
            self.path_callback,
            qos_reliable
        )
        
        # /local_plan - Local path from DWB planner
        self.local_plan_sub = self.create_subscription(
            Path,
            '/local_plan',
            self.local_plan_callback,
            qos_reliable
        )
        
        # /cmd_vel - Velocity commands
        self.vel_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.velocity_callback,
            qos_reliable
        )
        
        # /local_costmap/costmap - For obstacle detection info
        self.costmap_sub = self.create_subscription(
            OccupancyGrid,
            '/local_costmap/costmap',
            self.costmap_callback,
            qos_transient
        )
        
        # /joint_states - Wheel velocities from encoders
        self.joint_states_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_states_callback,
            qos_reliable
        )
        
        # /goal_pose - Navigation goal (single pose)
        self.goal_sub = self.create_subscription(
            PoseStamped,
            '/goal_pose',
            self.goal_callback,
            qos_reliable
        )
        
        # /pgv100_scan - Optical head QR/tag detection (if available)
        if HAS_OPTICAL_HEAD:
            self.pgv_sub = self.create_subscription(
                PgvScanData,
                '/optical_head/pgv100_scan',
                self.pgv_callback,
                qos_reliable
            )

        # ============================================================
        # PUBLISHERS - Custom AGV topics
        # ============================================================
        
        # /agv_pose - JSON formatted pose
        self.agv_pose_pub = self.create_publisher(String, '/agv_pose', 10)
        
        # /agv_path - JSON formatted path
        self.agv_path_pub = self.create_publisher(String, '/agv_path', 10)
        
        # /agv_local_path - JSON formatted local path
        self.agv_local_path_pub = self.create_publisher(String, '/agv_local_path', 10)
        
        # /agv_deviation - JSON formatted deviation (min distance amcl_pose -> local_plan)
        # NOTE: Deviation is calculated as the minimum distance from the robot's current pose 
        # to the closest point on the local path.
        self.agv_deviation_pub = self.create_publisher(String, '/agv_deviation', 10)
        
        # /agv_v - Velocity in m/s
        self.agv_vel_pub = self.create_publisher(Float32, '/agv_v', 10)
        
        # /agv_qr - QR/tag detection from optical head
        self.agv_qr_pub = self.create_publisher(String, '/agv_qr', 10)
        
        # /agv_detect - Object detection info
        self.agv_detect_pub = self.create_publisher(String, '/agv_detect', 10)
        
        # /wheel_v - Wheel velocities (JSON_WHEEL)
        self.wheel_v_pub = self.create_publisher(String, '/wheel_v', 10)
        
        # /agv_act - Movement/goals array (JSON_POSE array)
        self.agv_act_pub = self.create_publisher(String, '/agv_act', 10)
        
        # /agv_op - Operator AGV (JSON_OP)
        self.agv_op_pub = self.create_publisher(String, '/agv_op', 10)
        
        # ============================================================
        # NAV2 Action Client for goal monitoring
        # ============================================================
        self._nav_action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        # Timer for periodic status updates
        self.status_timer = self.create_timer(0.5, self.publish_agv_act)
        self.op_timer = self.create_timer(1.0, self.publish_agv_op)
        
        self.get_logger().info('NavTopicPublisher initialized')
        self.get_logger().info('Publishing: /agv_pose, /agv_path, /agv_local_path, /agv_deviation, /agv_v, /agv_qr, /agv_detect, /wheel_v, /agv_act, /agv_op')

    # ================================================================
    # CALLBACK FUNCTIONS
    # ================================================================
    
    def pose_callback(self, msg: PoseWithCovarianceStamped):
        """Convert AMCL pose to JSON format for /agv_pose"""
        self._latest_pose = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        
        pose_data = {
            "x": msg.pose.pose.position.x,
            "y": msg.pose.pose.position.y,
            "z": msg.pose.pose.position.z,
            "orientation": {
                "x": msg.pose.pose.orientation.x,
                "y": msg.pose.pose.orientation.y,
                "z": msg.pose.pose.orientation.z,
                "w": msg.pose.pose.orientation.w
            },
            "theta": self._quaternion_to_yaw(msg.pose.pose.orientation),
            "covariance": list(msg.pose.covariance),
            "frame_id": msg.header.frame_id,
            "timestamp": {
                "sec": msg.header.stamp.sec,
                "nanosec": msg.header.stamp.nanosec
            }
        }
        
        json_msg = String()
        json_msg.data = json.dumps(pose_data)
        self.agv_pose_pub.publish(json_msg)
        
    def path_callback(self, msg: Path):
        """Convert planned path to JSON format for /agv_path"""
        path_data = {
            "frame_id": msg.header.frame_id,
            "poses": []
        }
        
        for pose_stamped in msg.poses:
            path_data["poses"].append({
                "x": pose_stamped.pose.position.x,
                "y": pose_stamped.pose.position.y,
                "theta": self._quaternion_to_yaw(pose_stamped.pose.orientation)
            })
        
        json_msg = String()
        json_msg.data = json.dumps(path_data)
        self.agv_path_pub.publish(json_msg)
        
    def local_plan_callback(self, msg: Path):
        """
        Convert local plan to JSON format for /agv_local_path and 
        calculate deviation (minimum distance from robot to local path) for /agv_deviation.
        """
        import math
        
        local_path_data = {
            "frame_id": msg.header.frame_id,
            "poses": []
        }
        
        min_distance = float('inf')
        
        for pose_stamped in msg.poses:
            x = pose_stamped.pose.position.x
            y = pose_stamped.pose.position.y
            
            local_path_data["poses"].append({
                "x": x,
                "y": y,
                "theta": self._quaternion_to_yaw(pose_stamped.pose.orientation)
            })
            
            # Calculate distance to robot pose if available
            if self._latest_pose:
                dist = math.sqrt((x - self._latest_pose[0])**2 + (y - self._latest_pose[1])**2)
                if dist < min_distance:
                    min_distance = dist
        
        # Publish local path
        json_path_msg = String()
        json_path_msg.data = json.dumps(local_path_data)
        self.agv_local_path_pub.publish(json_path_msg)
        
        # Publish deviation if calculated
        if self._latest_pose and min_distance != float('inf'):
            deviation_data = {
                "deviation_m": round(min_distance, 4),
                "explanation": "min distance from /amcl_pose to /local_plan",
                "frame_id": msg.header.frame_id
            }
            json_dev_msg = String()
            json_dev_msg.data = json.dumps(deviation_data)
            self.agv_deviation_pub.publish(json_dev_msg)
            
    def velocity_callback(self, msg: Twist):
        """Extract linear velocity magnitude for /agv_v"""
        import math
        # Calculate velocity magnitude (m/s)
        vel_magnitude = math.sqrt(msg.linear.x**2 + msg.linear.y**2)
        
        vel_msg = Float32()
        vel_msg.data = vel_magnitude
        self.agv_vel_pub.publish(vel_msg)
        
    def costmap_callback(self, msg: OccupancyGrid):
        """Process costmap for obstacle detection info -> /agv_detect"""
        # Find cells with obstacles (value > 50 typically means obstacle)
        obstacle_count = sum(1 for cell in msg.data if cell > 50)
        total_cells = len(msg.data)
        obstacle_ratio = obstacle_count / total_cells if total_cells > 0 else 0.0
        
        detect_data = {
            "obstacle_detected": obstacle_count > 0,
            "obstacle_cell_count": obstacle_count,
            "obstacle_ratio": round(obstacle_ratio, 4),
            "resolution": msg.info.resolution,
            "width": msg.info.width,
            "height": msg.info.height,
            "frame_id": msg.header.frame_id
        }
        
        json_msg = String()
        json_msg.data = json.dumps(detect_data)
        self.agv_detect_pub.publish(json_msg)
        
    def joint_states_callback(self, msg: JointState):
        """Extract wheel velocities from joint states -> /wheel_v (JSON_WHEEL)"""
        wheel_data = {
            "wheels": {}
        }
        
        # Map joint names to velocities
        wheel_joints = [
            'wheel_front_left_joint',
            'wheel_front_right_joint', 
            'wheel_rear_left_joint',
            'wheel_rear_right_joint'
        ]
        
        for i, name in enumerate(msg.name):
            if name in wheel_joints or 'wheel' in name.lower():
                velocity = msg.velocity[i] if i < len(msg.velocity) else 0.0
                position = msg.position[i] if i < len(msg.position) else 0.0
                effort = msg.effort[i] if i < len(msg.effort) else 0.0
                
                # Clean up name for JSON key
                clean_name = name.replace('_joint', '').replace('wheel_', '')
                wheel_data["wheels"][clean_name] = {
                    "velocity": velocity,
                    "position": position,
                    "effort": effort
                }
        
        if wheel_data["wheels"]:
            json_msg = String()
            json_msg.data = json.dumps(wheel_data)
            self.wheel_v_pub.publish(json_msg)
        
    def pgv_callback(self, msg):
        """Convert optical head data to JSON for /agv_qr"""
        qr_data = {
            "tag_detected": bool(msg.tag_detected),
            "x": msg.x_pos,  # millimeters
            "y": msg.y_pos,  # millimeters
            "theta": msg.angle,  # degrees
            "tag_id": msg.tag_id if hasattr(msg, 'tag_id') else 0,
            "direction": msg.direction if hasattr(msg, 'direction') else "",
            "color_lane_count": msg.color_lane_count,
            "no_color_lane": bool(msg.no_color_lane),
            "no_pos": bool(msg.no_pos),
            "error": bool(msg.error) if hasattr(msg, 'error') else False,
            "frame_id": msg.header.frame_id
        }
        
        json_msg = String()
        json_msg.data = json.dumps(qr_data)
        self.agv_qr_pub.publish(json_msg)
        
    def goal_callback(self, msg: PoseStamped):
        """Handle navigation goal and update agv_act"""
        goal_pose = {
            "x": msg.pose.position.x,
            "y": msg.pose.position.y,
            "theta": self._quaternion_to_yaw(msg.pose.orientation),
            "frame_id": msg.header.frame_id
        }
        
        # Replace current goals with new single goal
        self._current_goals = [goal_pose]
        self._agv_operation["status"] = "NAVIGATING"
        self._agv_operation["command"] = "GOTO"
        
        # Immediately publish the update
        self.publish_agv_act()
        
    def publish_agv_act(self):
        """Publish agv_act - array of goal poses (JSON_POSE array)"""
        agv_act_data = {
            "goals": self._current_goals,
            "goal_count": len(self._current_goals),
            "status": self._agv_operation["status"]
        }
        
        json_msg = String()
        json_msg.data = json.dumps(agv_act_data)
        self.agv_act_pub.publish(json_msg)
        
    def publish_agv_op(self):
        """Publish agv_op - operator AGV status (JSON_OP)"""
        agv_op_data = {
            "status": self._agv_operation["status"],
            "command": self._agv_operation["command"],
            "is_navigating": self._agv_operation["status"] == "NAVIGATING",
            "is_idle": self._agv_operation["status"] == "IDLE",
            "has_goals": len(self._current_goals) > 0
        }
        
        json_msg = String()
        json_msg.data = json.dumps(agv_op_data)
        self.agv_op_pub.publish(json_msg)
        
    # ================================================================
    # HELPER FUNCTIONS
    # ================================================================
    
    def _quaternion_to_yaw(self, orientation):
        """Convert quaternion to yaw angle (radians)"""
        import math
        # Simplified quaternion to yaw conversion
        siny_cosp = 2 * (orientation.w * orientation.z + orientation.x * orientation.y)
        cosy_cosp = 1 - 2 * (orientation.y**2 + orientation.z**2)
        return math.atan2(siny_cosp, cosy_cosp)


def main(args=None):
    rclpy.init(args=args)
    node = NavTopicPublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
