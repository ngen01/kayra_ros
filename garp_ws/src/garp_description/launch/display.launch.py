#!/usr/bin/env python3
"""
GARP AMR - RViz'de Robot Görüntüleme Launch Dosyası
Robot modelini RViz'de görüntülemek için kullanılır.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # Paket yolları
    pkg_description = get_package_share_directory("garp_description")
    
    # URDF dosyası
    urdf_file = os.path.join(pkg_description, "urdf", "garp_robot.urdf.xacro")
    
    # Robot description (xacro işleme)
    robot_description = ParameterValue(
        Command(["xacro ", urdf_file]),
        value_type=str
    )

    # Robot State Publisher
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{
            "robot_description": robot_description,
            "use_sim_time": False
        }],
        output="screen"
    )

    # Joint State Publisher GUI
    joint_state_publisher_gui = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        output="screen"
    )

    # RViz
    rviz_config = os.path.join(pkg_description, "rviz", "display.rviz")
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", rviz_config] if os.path.exists(rviz_config) else [],
        output="screen"
    )

    return LaunchDescription([
        robot_state_publisher,
        joint_state_publisher_gui,
        rviz
    ])
