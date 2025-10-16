from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='task_manager_python',
            executable='task_manager_node',
            name='task_manager_node',
        ),
        Node(
            package='task_manager_python',
            executable='movement_controller_node',
            name='movement_controller_node',
        ),
        Node(
            package='object_chaser',
            executable='object_chaser_node',
            name='object_chaser_node',
        ),
    ])