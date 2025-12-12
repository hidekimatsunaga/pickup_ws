from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    params_file_arg = DeclareLaunchArgument(
        'object_chaser_params',
        default_value='/home/matsunaga-h/pickup_ws/src/object_chaser/config/object_chaser_params.yaml',
        description='Path to params YAML for object_chaser_node'
    )

    object_chaser = Node(
        package='object_chaser',
        executable='object_chaser_node',
        name='object_chaser_node',
        parameters=[LaunchConfiguration('object_chaser_params')]
    )

    return LaunchDescription([
        Node(
            package='task_manager',
            executable='task_manager_node',
            name='task_manager_node',
        ),
        Node(
            package='task_manager',
            executable='movement_controller_node',
            name='movement_controller_node',
        ),
        params_file_arg,
        object_chaser,
    ])