#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
# 3つの関節角度を配列で送るためのメッセージ型をインポート
# 実際のロボットに合わせて Float32MultiArray や自作メッセージ型に変更してください
from std_msgs.msg import Float64MultiArray 

class HoseControllerNode(Node):
    def __init__(self):
        super().__init__('hose_controller_node')

        # 現在のロボットの状態を保持する変数
        self.current_state = ""
        
        # Publisher: ホースの各セクションの目標角度を配信
        # トピック名やメッセージ型は、実際のホース制御機構に合わせてください
        self.hose_cmd_pub = self.create_publisher(Float64MultiArray, '/hose/target_angles', 10)

        # Subscriber: TaskManagerからロボットの状態を受信
        self.create_subscription(String, '/robot/state', self.state_callback, 10)
        
        # 回収動作中かどうかを示すフラグ
        self.is_collecting_motion_active = False

        self.get_logger().info("✅ Hose Controller ノード (ROS2) が起動しました。")

    def state_callback(self, msg):
        """
        /robot/state トピックを受信したときのコールバック関数
        """
        # 状態が変化した場合のみ処理を実行
        if self.current_state != msg.data:
            self.get_logger().info(f"ホースコントローラーが状態 [ {msg.data.UPPER()} ] を検知しました。")
            self.current_state = msg.data

            # 状態が 'collecting' になったら、回収動作を開始
            if self.current_state == "collecting":
                if not self.is_collecting_motion_active:
                    self.start_collection_motion()
            # それ以外の状態になったら、動作を停止して待機姿勢に戻す
            else:
                if self.is_collecting_motion_active:
                    self.stop_motion_and_reset()

    def start_collection_motion(self):
        """
        ゴミ回収のための一連のホース動作を開始する
        """
        self.get_logger().info("ホースの回収動作を開始します。")
        self.is_collecting_motion_active = True

        # TODO: ここにホースを動かす具体的なシーケンスを実装します。
        # 例として、目標のゴミの上までホースを伸ばす -> 吸着 -> ゴミ箱まで運ぶ、
        # という動作をタイマーを使って順番に実行するのが一般的です。

        # --- 以下は動作の簡単な例 ---
        # 1. まずゴミの位置にホースを伸ばす
        self.publish_hose_angles([45.0, 30.0, -20.0]) # 例の角度 [deg]
        
        # 2. N秒後に吸着を開始 (ここでは省略)
        # 3. M秒後にロボット上のゴミ箱までホースを移動
        # create_timer を使うと「N秒後にこの関数を実行」という実装が簡単にできます。

    def stop_motion_and_reset(self):
        """
        全ての動作を停止し、ホースを初期位置（待機姿勢）に戻す
        """
        self.get_logger().info("ホースの動作を停止し、待機姿勢に戻ります。")
        self.is_collecting_motion_active = False

        # 待機姿勢の角度を配信
        self.publish_hose_angles([0.0, 0.0, 0.0]) # 例: 全ての関節を0度にする

    def publish_hose_angles(self, angles_deg):
        """
        目標角度リストを指定してトピックに配信するヘルパー関数
        """
        msg = Float64MultiArray()
        msg.data = [float(angle) for angle in angles_deg] # Pythonのリストをセット
        self.hose_cmd_pub.publish(msg)
        self.get_logger().info(f"ホースへ指令角度: {msg.data} を送信しました。")


def main(args=None):
    rclpy.init(args=args)
    hose_controller_node = HoseControllerNode()
    try:
        rclpy.spin(hose_controller_node)
    except KeyboardInterrupt:
        pass
    finally:
        hose_controller_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()