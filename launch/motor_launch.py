from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
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
    ])