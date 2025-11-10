from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='hose_control',
            executable='line_goal_publisher',
            name='line_goal_publisher',
        ),
        Node(
            package='hose_control',
            executable='lookup_table',
            name='lookup_table',
        ),
    ])