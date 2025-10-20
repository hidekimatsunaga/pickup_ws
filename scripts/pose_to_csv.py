import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import csv
import os
from datetime import datetime

class PoseLogger(Node):
    def __init__(self):
        super().__init__('aruco_pose_logger_node')

        # パラメータを宣言（トピック名と出力ファイルパスを柔軟に変更可能にする）
        self.declare_parameter('topic_name', '/aruco/poses')
        self.declare_parameter('output_path', 'aruco_poses.csv')

        # パラメータを取得
        topic_name = self.get_parameter('topic_name').get_parameter_value().string_value
        output_path = self.get_parameter('output_path').get_parameter_value().string_value
        
        # --- ファイルの準備 ---
        try:
            # ファイルを書き込みモードで開く
            # newline='' は、CSVファイルで余分な空行が入るのを防ぐおまじない
            self.csv_file = open(output_path, 'w', newline='')
            self.csv_writer = csv.writer(self.csv_file)

            # ヘッダー行を書き込む
            header = [
                'timestamp_sec', 'timestamp_nanosec',
                'pos_x', 'pos_y', 'pos_z',
                'ori_x', 'ori_y', 'ori_z', 'ori_w'
            ]
            self.csv_writer.writerow(header)

            self.get_logger().info(f"'{output_path}' へのデータ記録を開始しました。")

        except IOError as e:
            self.get_logger().error(f"ファイルを開けませんでした: {e}")
            rclpy.shutdown()
            return

        # --- Subscriberの作成 ---
        self.subscription = self.create_subscription(
            PoseStamped,
            topic_name,
            self.pose_callback,
            10)
        
        # ノード終了時に呼ばれるクリーンアップ関数を登録
        # rclpy.on_shutdown(self.cleanup)

    def pose_callback(self, msg: PoseStamped):
        """
        PoseStampedメッセージを受け取ったときに呼び出されるコールバック関数
        """
        # メッセージからデータを抽出
        row = [
            msg.header.stamp.sec,
            msg.header.stamp.nanosec,
            msg.pose.position.x,
            msg.pose.position.y,
            msg.pose.position.z,
            msg.pose.orientation.x,
            msg.pose.orientation.y,
            msg.pose.orientation.z,
            msg.pose.orientation.w,
        ]
        
        # CSVファイルに1行書き込む
        self.csv_writer.writerow(row)

    def cleanup(self):
        """
        ノード終了時にファイルを閉じるための関数
        """
        if self.csv_file:
            self.csv_file.close()
            self.get_logger().info('ファイルを正常に保存して終了しました。')

def main(args=None):
    rclpy.init(args=args)
    
    pose_logger_node = PoseLogger()
    
    try:
        rclpy.spin(pose_logger_node)
    except KeyboardInterrupt:
        # Ctrl+Cが押されたときの処理
        pass
    finally:
        # ノードをシャットダウン
        pose_logger_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()