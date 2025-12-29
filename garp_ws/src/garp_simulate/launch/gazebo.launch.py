#!/usr/bin/env python3
"""
GARP AMR - Gazebo Simülasyon Launch Dosyası
Gazebo'da robotu spawn eder ve ROS köprüsünü başlatır.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # Paket yolları
    pkg_description = get_package_share_directory("garp_description")
    pkg_simulate = get_package_share_directory("garp_simulate")
    pkg_ros_gz_sim = get_package_share_directory("ros_gz_sim")

    # Launch argümanları
    world_arg = DeclareLaunchArgument(
        "world",
        default_value=os.path.join(pkg_simulate, "worlds", "empty_world.sdf"),
        description="Gazebo dünya dosyası"
    )
    
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Simülasyon zamanı kullan"
    )

    # Gazebo model path'i ayarla
    gz_model_path = SetEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH",
        value=os.path.join(pkg_simulate, "models") + ":" + 
              os.path.join(pkg_simulate, "worlds")
    )

    # URDF dosyası
    urdf_file = os.path.join(pkg_description, "urdf", "garp_robot.urdf.xacro")
    robot_description = ParameterValue(
        Command(["xacro ", urdf_file]),
        value_type=str
    )

    # Gazebo Sim başlat
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, "launch", "gz_sim.launch.py")
        ),
        launch_arguments={
            "gz_args": ["-r -v 4 ", LaunchConfiguration("world")]
        }.items()
    )

    # Robot State Publisher
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{
            "robot_description": robot_description,
            "use_sim_time": LaunchConfiguration("use_sim_time")
        }],
        output="screen"
    )

    # Gazebo'ya robot spawn et
    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-topic", "robot_description",
            "-name", "garp_amr",
            "-x", "0.0",
            "-y", "0.0",
            "-z", "0.1"
        ],
        output="screen"
    )

    # ROS-Gazebo Bridge
    bridge_config = os.path.join(pkg_simulate, "config", "gz_bridge.yaml")
    ros_gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        parameters=[{"config_file": bridge_config}],
        output="screen"
    )

    # RViz
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", os.path.join(pkg_simulate, "config", "gazebo.rviz")],
        output="screen"
    )

    return LaunchDescription([
        world_arg,
        use_sim_time_arg,
        gz_model_path,
        gazebo,
        robot_state_publisher,
        spawn_robot,
        ros_gz_bridge,
        rviz
    ])
