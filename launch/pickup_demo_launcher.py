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
            package='task_manager',
            executable='vacuum_manager_node',
            name='vacuum_manager_node',
        ),
        Node(
            package='task_manager',
            executable='manipulator_manager_node',
            name='manipulator_manager',
        ),
        Node(
            package='serial_transciever',
            executable='angle_serial_node',
            name='angle_serial_node',
        ),
        Node(
            package='serial_transciever',
            executable='chokudo_cameraswing_air_serial_node',
            name='chokudo_cameraswing_air_serial_node',
        ),
        Node(
            package='target_selector',
            executable='target_selector_node_exe',
            name='target_selector_node',
        ),
        Node(
            package='hose_control',
            executable='lookup_table',
            name='lookup_table',
        ),
    ])