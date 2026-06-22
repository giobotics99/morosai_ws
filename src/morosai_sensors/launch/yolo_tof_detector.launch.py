"""
Launch file for the YOLO ToF Detector node.

Usage:
  ros2 launch morosai_sensors yolo_tof_detector.launch.py
  ros2 launch morosai_sensors yolo_tof_detector.launch.py confidence_threshold:=0.5
  ros2 launch morosai_sensors yolo_tof_detector.launch.py device:=cuda:0
"""

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_dir = get_package_share_directory("morosai_sensors")

    # Default weights path (resolved at share install time)
    default_weights = os.path.join(pkg_dir, "..", "..", "..", "..",
                                   "src", "morosai_sensors", "weights", "yolo26n.pt")
    default_weights = os.path.normpath(default_weights)

    return LaunchDescription([
        # ── Tuneable launch arguments ──────────────────────────────────────────
        DeclareLaunchArgument(
            "weights_path",
            default_value=default_weights,
            description="Absolute path to the YOLO .pt weights file",
        ),
        DeclareLaunchArgument(
            "image_topic",
            default_value="/gordon_tof/bgr",
            description="BGR image topic from the ToF camera",
        ),
        DeclareLaunchArgument(
            "confidence_threshold",
            default_value="0.20",
            description="Minimum confidence score [0.0–1.0]",
        ),
        DeclareLaunchArgument(
            "iou_threshold",
            default_value="0.25",
            description="IoU threshold for NMS suppression",
        ),
        DeclareLaunchArgument(
            "device",
            default_value="cpu",
            description="Inference device: 'cpu' or 'cuda:0'",
        ),
        DeclareLaunchArgument(
            "imgsz",
            default_value="640",
            description="YOLO inference resolution (square side, px)",
        ),

        # ── YOLO Detector Node ─────────────────────────────────────────────────
        Node(
            package="morosai_sensors",
            executable="yolo_tof_detector.py",
            name="yolo_tof_detector",
            output="screen",
            parameters=[{
                "weights_path":         LaunchConfiguration("weights_path"),
                "image_topic":          LaunchConfiguration("image_topic"),
                "confidence_threshold": LaunchConfiguration("confidence_threshold"),
                "iou_threshold":        LaunchConfiguration("iou_threshold"),
                "target_classes":       ["person", "box", "chair"],
                "device":               LaunchConfiguration("device"),
                "imgsz":                LaunchConfiguration("imgsz"),
                "publish_annotated_image": True,
            }],
        ),
    ])
