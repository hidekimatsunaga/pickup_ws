from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    object_chaser_params_arg = DeclareLaunchArgument(
        'object_chaser_params',
        default_value='/home/matsunaga-h/pickup_ws/src/object_chaser/config/object_chaser_params.yaml',
        description='Path to params YAML for object_chaser_node'
    )

    task_manager_params_arg = DeclareLaunchArgument(
        'task_manager_params',
        default_value='/home/matsunaga-h/pickup_ws/src/manager/task_manager/config/task_manager_params.yaml',
        description='Path to params YAML for task_manager and movement_controller'
    )

    object_chaser = Node(
        package='object_chaser',
        executable='object_chaser_node',
        name='object_chaser_node',
        parameters=[LaunchConfiguration('object_chaser_params')]
    )

    task_manager = Node(
        package='task_manager',
        executable='task_manager_node',
        name='task_manager_node',
        parameters=[LaunchConfiguration('task_manager_params')]
    )

    movement_controller = Node(
        package='task_manager',
        executable='movement_controller_node',
        name='movement_controller_node',
        parameters=[LaunchConfiguration('task_manager_params')]
    )

    return LaunchDescription([
        object_chaser_params_arg,
        task_manager_params_arg,
        task_manager,
        movement_controller,
        object_chaser,
    ])