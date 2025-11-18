#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Float32
import csv
import time

# このCSVファイルは、スクリプト実行時に同じディレクトリにある必要があります
CSV_FILE_PATH = '/home/matsunaga-h/pickup_ws/angle_arucopose_csv/aruco_motor_log_1108_175143.csv'

class CsvPlayerNode(Node):
    def __init__(self):
        super().__init__('csv_motor_player')
        
        # パブリッシャーの作成
        # Topic 1: motor1 ~ motor9 (Float32MultiArray)
        self.joint_pub = self.create_publisher(
            Float32MultiArray,
            '/motor_angles',
            10
        )

        # Topic 2: motor10 (Float32)
        self.choku_pub = self.create_publisher(
            Float32,
            '/chokudomotor/target_angle',
            10
        )
        
        self.get_logger().info(f"CSVプレーヤーノードが起動しました。")
        self.get_logger().info(f"読み込むファイル: {CSV_FILE_PATH}")

    def play_csv(self):
        self.get_logger().info("CSVの再生を開始します...")
        
        try:
            with open(CSV_FILE_PATH, 'r') as f:
                # CSVファイルを辞書形式で読み込む
                reader = csv.DictReader(f)
                
                last_timestamp = None
                
                # Float32MultiArrayメッセージは一度作成し、中身を更新していく
                joint_state_msg = Float32MultiArray()

                # Float32メッセージ（chokudomotor用）
                choku_msg = Float32()

                for i, row in enumerate(reader):
                    try:
                        # 1. タイムスタンプを読み込み、再生タイミングを計算
                        current_timestamp = float(row['timestamp'])
                        
                        if last_timestamp is not None:
                            # 前の行からの経過時間だけ待機
                            delay = current_timestamp - last_timestamp
                            if delay > 0:
                                time.sleep(delay)
                        
                        last_timestamp = current_timestamp

                        # 2. Topic 1 (/motor_current_angles) のデータを準備
                        positions = [
                            float(row['motor1']), float(row['motor2']),
                            float(row['motor3']), float(row['motor4']),
                            float(row['motor5']), float(row['motor6']),
                            float(row['motor7']), float(row['motor8']),
                            float(row['motor9'])
                        ]
                        
                        # Float32MultiArrayにデータをセット
                        joint_state_msg.data = positions
                        
                        # 3. Topic 2 (/chokudomotor_angle) のデータを準備
                        choku_msg.data = float(row['motor10'])
                        
                        # 4. パブリッシュ
                        self.joint_pub.publish(joint_state_msg)
                        self.choku_pub.publish(choku_msg)

                        if i % 100 == 0: # 100行ごとにログを出力
                           self.get_logger().info(f"行 {i}: タイムスタンプ {current_timestamp} のデータを送信")

                    except (ValueError, KeyError) as e:
                        self.get_logger().warn(f"行 {i} の読み込みエラー: {e}。スキップします。")
                    except Exception as e:
                        self.get_logger().error(f"予期せぬエラー: {e}")
                        break
                        
        except FileNotFoundError:
            self.get_logger().error(f"エラー: ファイル '{CSV_FILE_PATH}' が見つかりません。")
            return
        except Exception as e:
            self.get_logger().error(f"CSVファイルを開けません: {e}")
            return

        self.get_logger().info("CSVファイルの再生が完了しました。")


def main(args=None):
    rclpy.init(args=args)
    node = CsvPlayerNode()
    
    try:
        # スピン(spin)の代わりに、再生関数を直接呼び出す
        # 再生が完了したらノードは終了する
        node.play_csv()
    except KeyboardInterrupt:
        node.get_logger().info("再生が中断されました。")
    finally:
        # クリーンアップ
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()