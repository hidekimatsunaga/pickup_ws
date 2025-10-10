from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # 起動したいノードのリストを定義
    # (実行ファイル名, ノード名) のタプルのリスト
    nodes_to_launch = [
        ('angle_serial_node', 'angle_serial_node'),
        ('chokudo_cameraswing_air_serial_node', 'angle_chokudo_node'),
        ('motor_manual_chokudo_node', 'motor_manual_node'),
        ('angle_arucopose_csv', 'angle_arucopose_csv'),
    ]

    # forループを使ってNodeアクションを生成
    launch_nodes = []
    for executable, name in nodes_to_launch:
        launch_nodes.append(
            Node(
                package='serial_transciever',
                executable=executable,
                name=name,
            )
        )

    return LaunchDescription(launch_nodes)