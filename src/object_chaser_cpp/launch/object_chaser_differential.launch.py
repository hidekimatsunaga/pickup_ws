from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('object_chaser_cpp'),
        'config',
        'object_chaser_differential_params.yaml'
    )

    return LaunchDescription([
        Node(
            package='object_chaser_cpp',
            executable='object_chaser_node_differential',
            name='object_chaser_node_differential',
            output='screen',
            parameters=[config]
        )
    ])
