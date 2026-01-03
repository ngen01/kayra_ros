import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.substitutions import Command
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    pkg_share = get_package_share_directory("garp_description")
    urdf_path = os.path.join(pkg_share, "urdf", "main.urdf.xacro")
    gz_bridge_cfg = os.path.join(pkg_share, "config", "gz_bridge.yaml")
    
    robot_desc = ParameterValue(Command(["xacro ", urdf_path]), value_type=str)

    return LaunchDescription([
        # Gazebo Sim
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('ros_gz_sim'),
                    'launch', 'gz_sim.launch.py'
                )
            ),
            launch_arguments={'gz_args': '-r -v 4 empty.sdf'}.items()
        ),
        
        # Robot State Publisher
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[{"robot_description": robot_desc}]
        ),
        
        # Spawn Robot
        Node(
            package='ros_gz_sim',
            executable='create',
            parameters=[{'topic': 'robot_description'}],
        ),
        
        # ROS-Gazebo Bridge
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            parameters=[{'config_file': gz_bridge_cfg}],
        ),
        
        # RViz
        Node(
            package="rviz2",
            executable="rviz2"
        ),
        Node(
            package='garp_tools',
            executable='odom_to_tf',
            parameters=[{
            'odom_topic': 'odom',
            'odom_frame': 'odom_frame',
            'base_frame': 'base_footprint'}]
        ),
    ])