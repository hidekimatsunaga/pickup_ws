from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='target_selector',
            executable='target_selector_node_exe',
            name='target_selector_node',
        ),
        Node(
            package='hose_control',
            executable='feedback_goal_position_node',
            name='feedback_goal_position_node',
        ),
        Node(
            package='hose_control',
            executable='lookup_table',
            name='lookup_table',
        ),
    ])