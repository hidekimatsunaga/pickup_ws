from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('hose_control')
    lookup_params = os.path.join(pkg_share, 'config', 'lookup_table_params.yaml')

    return LaunchDescription([
        Node(
            package='hose_control',
            executable='feedback_2_node',
            name='feedback_2_node',
        ),
        Node(
            package='hose_control',
            executable='lookup_table',
            name='feedback_motor_publisher',
            output='screen',
            parameters=[lookup_params]
        ),
    ])