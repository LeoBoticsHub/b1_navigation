from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os

def generate_launch_description():

     # * ----- Pointcloud to Laserscan -----

    # Pointcloud_to_laserscan Node
    pcd_to_lsr = Node(
        package='pointcloud_to_laserscan', 
        executable='pointcloud_to_laserscan_node',
        remappings=[('cloud_in', LaunchConfiguration('pcd_input')),
                    ('scan', LaunchConfiguration('scan_output'))],
        parameters=[{
            'target_frame': LaunchConfiguration('lidar_frame'),
            'transform_tolerance': 0.01,
            'min_height': -0.7,
            'max_height': 0.3,
            'angle_min': -3.14, # M_PI
            'angle_max': 3.14, # M_PI
            'angle_increment': 0.0087,  # M_PI/360.0
            'scan_time': 0.3333,
            'range_min': 0.45,
            'range_max': 60.0,
            'use_inf': True,
            'inf_epsilon': 1.0
        }],
        name='pointcloud_to_laserscan'
    )

    # * ----- localization -----

    map_file = LaunchConfiguration("map_file")
    use_sim_time = LaunchConfiguration("use_sim_time", default=True)

    lifecycle_node_names = ["map_server", "amcl"]

    localization_params = os.path.join(get_package_share_directory("b1_navigation"), 'config', 'localization.yaml')

    load_localization_nodes = GroupAction([
        Node(
            package="nav2_map_server",
            executable="map_server",
            name="map_server",
            output="screen",
            parameters=[
                localization_params,
                {"yaml_filename": map_file},
            ],
        ),
        Node(
            package="nav2_amcl",
            executable="amcl",
            name="amcl",
            output="screen",
            parameters=[localization_params],
        ),
        Node(
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
    ])
    
    return LaunchDescription([
        DeclareLaunchArgument(
            name='pcd_input', default_value='lidar_points',
            description='Name of point cloud in input'
        ),
        DeclareLaunchArgument(
            name='scan_output', default_value='scan',
            description='Name of laserscan in output'
        ),
        DeclareLaunchArgument(
            name='lidar_frame', default_value='hesai_lidar',
            description='Name of the lidar tf'
        ),
        pcd_to_lsr,
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