from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
import launch_ros.actions
import os

def generate_launch_description():
    return LaunchDescription([
        launch_ros.actions.Node(
            package='robot_localization',
            executable='ekf_node',
            name='odom_node',
            output='screen',
            parameters=[os.path.join(get_package_share_directory("b1_navigation"), 'config', 'odometry.yaml')],
           ),
])
