import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():

    # ---- Paths ----
    nav2_pkg = get_package_share_directory("nav2_bringup")
    sim_pkg = get_package_share_directory("garp_simulate")

    nav2_params = os.path.join(
        sim_pkg,
        "config",
        "nav2_params.yaml"
    )

    map_yaml = LaunchConfiguration("map")

    # ---- Launch Arguments ----
    declare_map = DeclareLaunchArgument(
        "map",
        default_value=os.path.join(sim_pkg, "maps", "in_my_house.yaml"),
        description="Full path to map yaml file"
    )

    return LaunchDescription([

        declare_map,

        # --------------------------------------------------
        # ODOM -> TF
        # --------------------------------------------------
        Node(
            package="garp_tools",
            executable="odom_to_tf",
            name="odom_to_tf",
            output="screen",
            parameters=[{
                "use_sim_time": True,
                "odom_topic": "garp/odom"
            }]
        ),

        # --------------------------------------------------
        # MAP SERVER
        # --------------------------------------------------
        Node(
            package="nav2_map_server",
            executable="map_server",
            name="map_server",
            output="screen",
            parameters=[{
                "use_sim_time": True,
                "yaml_filename": map_yaml
            }]
        ),

        # --------------------------------------------------
        # AMCL
        # --------------------------------------------------
        Node(
            package="nav2_amcl",
            executable="amcl",
            name="amcl",
            output="screen",
            parameters=[{
                "use_sim_time": True,
                "global_frame_id": "map",
                "odom_frame_id": "odom_frame",
                "base_frame_id": "base_footprint",
                "scan_topic": "lidar/scan"
            }]
        ),

        # --------------------------------------------------
        # LIFECYCLE MANAGER (map_server + amcl)
        # --------------------------------------------------
        Node(
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            name="lifecycle_manager_localization",
            output="screen",
            parameters=[{
                "use_sim_time": True,
                "autostart": True,
                "node_names": ["map_server", "amcl"]
            }]
        ),

        # --------------------------------------------------
        # NAV2 NAVIGATION
        # --------------------------------------------------
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    nav2_pkg,
                    "launch",
                    "navigation_launch.py"
                )
            ),
            launch_arguments={
                "use_sim_time": "true",
                "params_file": nav2_params
            }.items()
        ),
    ])
