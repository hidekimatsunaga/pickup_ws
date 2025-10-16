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
            package='task_manager',
            executable='vacuum_manager_node',
            name='vacuum_manager_node',
        ),
        Node(
            package='task_manager',
            executable='manipulator_manager',
            name='manipulator_manager',
        ),
    ])