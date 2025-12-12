from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value='/home/matsunaga-h/pickup_ws/src/object_chaser/config/object_chaser_params.yaml',
        description='Path to ROS2 params YAML for object_chaser_node'
    )

    chaser_node = Node(
        package='object_chaser',
        executable='object_chaser_node',
        name='object_chaser_node',
        output='screen',
        parameters=[LaunchConfiguration('params_file')]
    )

    return LaunchDescription([
        params_file_arg,
        chaser_node
    ])
