#!/usr/bin/python3
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

def generate_launch_description():
    cfg = os.path.join(get_package_share_directory("fast_livo"), "config")
    rviz_cfg = os.path.join(get_package_share_directory("fast_livo"), "rviz_cfg", "fast_livo2.rviz")
    main_cfg   = os.path.join(cfg, "mid360_hikrobot.yaml")
    camera_cfg = os.path.join(cfg, "camera_pinhole_mid360.yaml")

    return LaunchDescription([
        DeclareLaunchArgument("use_rviz", default_value="False"),

        # Camera intrinsics served via parameter_blackboard
        Node(
            package='demo_nodes_cpp',
            executable='parameter_blackboard',
            name='parameter_blackboard',
            parameters=[camera_cfg],
            output='screen'
        ),

        Node(
            package="fast_livo",
            executable="fastlivo_mapping",
            name="laserMapping",
            parameters=[main_cfg],
            output="screen"
        ),

        # RViz2 — fix libusb crash first:
        # sudo apt install libusb-1.0-0-dev && sudo ldconfig
        Node(
            condition=IfCondition(LaunchConfiguration("use_rviz")),
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            arguments=["-d", rviz_cfg],
            output="screen"
        ),
    ])
