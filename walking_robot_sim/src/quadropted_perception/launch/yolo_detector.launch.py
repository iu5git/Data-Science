#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory("quadropted_perception")

    return LaunchDescription([
        DeclareLaunchArgument("model_name", default_value="yolov8n.pt"),
        DeclareLaunchArgument("model_path", default_value=""),
        DeclareLaunchArgument("confidence_threshold", default_value="0.5"),
        DeclareLaunchArgument("camera_topic", default_value="/robot1/color/image_raw"),
        DeclareLaunchArgument("device", default_value="cpu"),

        Node(
            package="quadropted_perception",
            executable="yolo_detector",
            name="yolo_detector",
            output="screen",
            parameters=[os.path.join(pkg_dir, "config", "yolo_detector.yaml")],
        ),

        Node(
            package="quadropted_perception",
            executable="visualizer",
            name="detection_visualizer",
            output="screen",
        ),
    ])
