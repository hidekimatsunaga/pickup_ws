from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='serial_transciever',
            executable='relay_controller',
            name='relay_controller',
        ),
        Node(
            package='serial_transciever',
            executable='flag_relay_bridge',
            name='flag_relay_bridge',
        ),
        # Node(
        #     package='task_manager',
        #     executable='vacuum_manager_node',
        #     name='vacuum_manager_node',
        # ),
    ])