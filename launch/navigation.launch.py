from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    OpaqueFunction
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch.conditions import IfCondition


ARGUMENTS = [
    DeclareLaunchArgument('use_sim_time', default_value='false',
                          choices=['true', 'false'],
                          description='Use sim time'),
    DeclareLaunchArgument('mapping', default_value='false',
                          choices=['true', 'false'],
                          description='Use sim time'),
    DeclareLaunchArgument('map_file', default_value='map',
                          description='The name of the map to load with map server'),
    DeclareLaunchArgument('localization', default_value='false',
                          choices=['true', 'false'],
                          description='Use sim time'),
    DeclareLaunchArgument('params_file',
                          default_value=PathJoinSubstitution([
                            get_package_share_directory('b1_navigation'),
                            'config',
                            'navigation.yaml'
                          ]),
                          description='Nav2 parameters')
]


def launch_setup(context, *args, **kwargs):

    nav2_params = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    map_file = LaunchConfiguration('map_file')

    launch_nav2 = PathJoinSubstitution(
        [get_package_share_directory('nav2_bringup'), 'launch', 'navigation_launch.py']
    )

    launch_mapping = PathJoinSubstitution(
        [get_package_share_directory('b1_navigation'), 'launch', 'mapping.launch.py']
    )

    launch_localization = PathJoinSubstitution(
        [get_package_share_directory('b1_navigation'), 'launch', 'localization.launch.py']
    )

    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(launch_nav2),
        launch_arguments=[
            ('use_sim_time', use_sim_time),
            ('params_file', nav2_params.perform(context)),
            ('use_composition', 'False')
        ]
    )
    
    mapping = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(launch_mapping),
        condition=IfCondition(LaunchConfiguration('mapping'))
    )

    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(launch_localization),
        condition=IfCondition(
            PythonExpression(
                ["'", LaunchConfiguration('localization'), "' == 'true' and '", LaunchConfiguration('mapping'), "' == 'false'"]
            )
        ),
        launch_arguments=[
            ('use_sim_time', use_sim_time),
            ('map_file', map_file)
        ]
    )

    nav2 = GroupAction([
        navigation,
        mapping,
        localization
    ])

    return [nav2]


def generate_launch_description():
    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(OpaqueFunction(function=launch_setup))
    return ld