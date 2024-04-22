from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition, UnlessCondition
import os

def generate_launch_description():

    # * ----- Pointcloud to Laserscan -----

    # Arguments
    pcd_input_arg = DeclareLaunchArgument(
        name='pcd_input', default_value='lidar_points',
        description='Name of point cloud in input'
    )
    scan_output_arg = DeclareLaunchArgument(
        name='scan_output', default_value='scan',
        description='Name of laserscan in output'
    )
    lidar_frame = DeclareLaunchArgument(
        name='lidar_frame', default_value='hesai_lidar',
        description='Name of the lidar tf'
    )
    
    # Pointcloud_to_laserscan Node
    pcd_to_lsr = Node(
        package='pointcloud_to_laserscan', 
        executable='pointcloud_to_laserscan_node',
        remappings=[('cloud_in', LaunchConfiguration('pcd_input')),
                    ('scan', LaunchConfiguration('scan_output'))],
        parameters=[{
            'target_frame': LaunchConfiguration('lidar_frame'),
            'transform_tolerance': 0.01,
            'min_height': 0.0,
            'max_height': 1.0,
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

    # * ----- Slam toolbox mapping -----

    ARGUMENTS = [
    DeclareLaunchArgument('use_sim_time', default_value='false',
                          choices=['true', 'false'],
                          description='Use sim time'),
    DeclareLaunchArgument('sync', default_value='true',
                          choices=['true', 'false'],
                          description='Use synchronous SLAM')#,
    # DeclareLaunchArgument('namespace', default_value='',
    #                       description='Robot namespace')
    ]

    # namespace = LaunchConfiguration('namespace')
    sync = LaunchConfiguration('sync')

    remappings = [
        ('/tf', 'tf'),
        ('/tf_static', 'tf_static'),
        ('/scan', 'scan'),
        ('/map', 'map'),
        ('/map_metadata', 'map_metadata'),
    ]

    slam = GroupAction([
        # PushRosNamespace(namespace),

        Node(package='slam_toolbox',
             executable='sync_slam_toolbox_node',
             name='slam_toolbox',
             output='screen',
             parameters=[
               os.path.join(get_package_share_directory("b1_navigation"), 'config', 'mapping.yaml'),
               {'use_sim_time': LaunchConfiguration('use_sim_time')}
             ],
             remappings=remappings,
             condition=IfCondition(sync)),

        Node(package='slam_toolbox',
             executable='async_slam_toolbox_node',
             name='slam_toolbox',
             output='screen',
             parameters=[
               os.path.join(get_package_share_directory("b1_navigation"), 'config', 'mapping.yaml'),
               {'use_sim_time': LaunchConfiguration('use_sim_time')}
             ],
             remappings=remappings,
             condition=UnlessCondition(sync))
    ])

    ld = LaunchDescription(ARGUMENTS)
    # ld.add_action(slam_params_arg)
    ld.add_action(slam)
    # return ld


    # * ----- Launch description -----

    # ld = LaunchDescription()
    ld.add_action(pcd_input_arg)
    ld.add_action(scan_output_arg)
    ld.add_action(lidar_frame)
    ld.add_action(pcd_to_lsr)

    return ld