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
        Node(
            package='hose_control',
            executable='flag_manager',
            name='flag_manager',
        ),
        Node(
            package='serial_transciever',
            executable='angle_serial_node',
            name='angle_serial_node',
        ),
        Node(
            package='serial_transciever',
            executable='chokudo_cameraswing_air_serial_node',
            name='angle_chokudo_node',
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