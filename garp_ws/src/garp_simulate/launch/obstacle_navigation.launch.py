#!/usr/bin/env python3
"""
GARP AMR - Obstacle Navigation Launch
Engelli dünya + Nav2 navigasyon entegrasyonu.
Tek komutla tüm sistemi başlatır.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # Paket yolları
    pkg_description = get_package_share_directory("garp_description")
    pkg_simulate = get_package_share_directory("garp_simulate")
    pkg_ros_gz_sim = get_package_share_directory("ros_gz_sim")
    pkg_nav2_bringup = get_package_share_directory("nav2_bringup")

    # Dosya yolları
    world_file = os.path.join(pkg_simulate, "worlds", "obstacle_world.sdf")
    urdf_file = os.path.join(pkg_description, "urdf", "garp_robot.urdf.xacro")
    bridge_config = os.path.join(pkg_simulate, "config", "gz_bridge.yaml")
    nav2_params = os.path.join(pkg_simulate, "config", "nav2_params.yaml")
    slam_params = os.path.join(pkg_simulate, "config", "slam_toolbox.yaml")

    # Argümanlar
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time", default_value="true"
    )
    
    slam_arg = DeclareLaunchArgument(
        "slam", default_value="true",
        description="SLAM modu (harita oluşturma)"
    )
    
    map_arg = DeclareLaunchArgument(
        "map", default_value="",
        description="Harita dosyası (SLAM kapalıysa)"
    )

    # Gazebo model path
    gz_model_path = SetEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH",
        value=os.path.join(pkg_simulate, "models") + ":" + 
              os.path.join(pkg_simulate, "worlds")
    )

    # Robot description
    robot_description = ParameterValue(
        Command(["xacro ", urdf_file]),
        value_type=str
    )

    # ==================== GAZEBO ====================
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, "launch", "gz_sim.launch.py")
        ),
        launch_arguments={
            "gz_args": f"-r -v 4 {world_file}"
        }.items()
    )

    # Robot State Publisher
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{
            "robot_description": robot_description,
            "use_sim_time": True
        }],
        output="screen"
    )

    # Spawn Robot
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
    ros_gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        parameters=[{"config_file": bridge_config}],
        output="screen"
    )

    # ==================== SLAM ====================
    slam_toolbox = Node(
        package="slam_toolbox",
        executable="async_slam_toolbox_node",
        name="slam_toolbox",
        output="screen",
        parameters=[
            slam_params,
            {"use_sim_time": True}
        ]
    )

    # ==================== NAV2 ====================
    # Nav2 bringup (SLAM modunda map_server kullanmıyor)
    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2_bringup, "launch", "navigation_launch.py")
        ),
        launch_arguments={
            "params_file": nav2_params,
            "use_sim_time": "true"
        }.items()
    )

    # ==================== RVIZ ====================
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        output="screen"
    )

    # Nav2 ve SLAM biraz geciktirilsin (Gazebo ve robot hazır olsun)
    delayed_slam = TimerAction(
        period=5.0,
        actions=[slam_toolbox]
    )

    delayed_nav2 = TimerAction(
        period=7.0,
        actions=[nav2_bringup]
    )

    return LaunchDescription([
        use_sim_time_arg,
        slam_arg,
        map_arg,
        gz_model_path,
        
        # Gazebo + Robot
        gazebo,
        robot_state_publisher,
        spawn_robot,
        ros_gz_bridge,
        
        # SLAM (5 saniye sonra)
        delayed_slam,
        
        # Nav2 (7 saniye sonra)
        delayed_nav2,
        
        # RViz
        rviz
    ])
