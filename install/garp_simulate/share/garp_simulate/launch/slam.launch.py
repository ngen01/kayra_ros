import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory("garp_simulate")
    cfg_path = os.path.join(pkg_share, "config")
    slam_tb_params_cfg = os.path.join(cfg_path, "stb_localization.yaml")

    return LaunchDescription([
        Node(
            package="garp_tools",
            executable="odom_to_tf",
            parameters=[{"odom_topic": "/garp/odom"}]
        ),

        # ---- SLAM Toolbox (only onmap=true) ----
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory("slam_toolbox"),
                    "launch",
                    "online_async_launch.py"
                )
            ),
            launch_arguments={
                "slam_params_file": slam_tb_params_cfg,
            }.items()
        )
    ])