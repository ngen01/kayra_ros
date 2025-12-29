#!/usr/bin/env python3
"""
GARP AMR - SLAM Launch
GARP SLAM launch dosyası.
odom_to_tf + slam_toolbox + RViz
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("garp_simulate")
    cfg_path = os.path.join(pkg_share, "config")

    slam_params = os.path.join(cfg_path, "slam_toolbox.yaml")
    rviz_config = os.path.join(cfg_path, "mapping.rviz")

    return LaunchDescription([
        # ODOM -> TF
        Node(
            package="garp_scripts",
            executable="odom_to_tf",
            name="odom_to_tf",
            output="screen",
            parameters=[{
                "use_sim_time": True,
                "odom_topic": "garp/odom",
                "odom_frame": "odom_frame",
                "base_frame": "base_footprint"
            }]
        ),

        # SLAM Toolbox
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory("slam_toolbox"),
                    "launch",
                    "online_async_launch.py"
                )
            ),
            launch_arguments={
                "slam_params_file": slam_params,
            }.items()
        ),

        # RViz
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
            arguments=["-d", rviz_config]
        )
    ])
