from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os

def generate_launch_description():
    map_file = LaunchConfiguration("map_file")
    use_sim_time = LaunchConfiguration("use_sim_time")

    lifecycle_node_names = ["map_server", "amcl"]

    localization_params = os.path.join(get_package_share_directory("b1_navigation"), 'config', 'localization.yaml')

    load_localization_nodes = GroupAction(
        map_server = Node(
            package="nav2_map_server",
            executable="map_server",
            name="map_server",
            output="screen",
            parameters=[
                localization_params,
                {"yaml_filename": map_file},
            ],
        ),
        amcl = Node(
            package="nav2_amcl",
            executable="amcl",
            name="amcl",
            output="screen",
            parameters=[localization_params],
        ),
        lifecycle = Node(
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            name="lifecycle_manager_localization",
            output="screen",
            parameters=[
                {
                    "use_sim_time": use_sim_time,
                    "node_names": lifecycle_node_names,
                    "autostart": True,
                }
            ],
        )
    )
    
    return LaunchDescription([
            DeclareLaunchArgument(
                "map_file", description="Full path to map yaml file to load"
            ),
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="true",
                description="Whether to use simulation (Gazebo) clock",
            ),
            load_localization_nodes
    ])