#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist

class MovementController(Node):
    def __init__(self):
        super().__init__('movement_controller_node')

        # 現在のロボットの状態を保持する変数
        self.current_state = ""
        
        # Publisher: ロボットの速度指令を配信
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Subscriber: TaskManagerからロボットの状態を受信
        self.create_subscription(String, '/robot/state', self.state_callback, 10)

        # 0.1秒ごとに速度指令を出し続けるためのタイマー
        self.timer = self.create_timer(0.1, self.publish_cmd_vel)

        self.get_logger().info("✅ Movement Controller ノード (ROS2) が起動しました。")

    def state_callback(self, msg):
        """
        /robot/state トピックを受信したときのコールバック関数
        """
        # 状態が変化した場合のみログを出力
        if self.current_state != msg.data:
            self.get_logger().info(f"状態が [ {msg.data.upper()} ] になりました。移動制御を更新します。")
            self.current_state = msg.data

    def publish_cmd_vel(self):
        """
        現在の状態に基づいて Twist メッセージを生成し、/cmd_vel に配信する
        """
        twist = Twist() # Twistメッセージを初期化（中身は全て0）

        # 状態に応じて速度を決定
        if self.current_state == "searching":
            # 探索中：ゆっくり前進
            twist.linear.x = 0.1  # 前進速度 [m/s]
            twist.angular.z = 0.0 # 角速度 [rad/s]

        elif self.current_state == "approaching":
            # 接近中：目標に向かって前進
            # TODO: 本来は、ここで検出した物体の位置情報 (/detected_objects_3dなど) をもとに
            #       目標地点へのx方向の速度とω（角速度）を計算する必要があります。
            #       今回は仮の動作として、少し速めに直進します。
            twist.linear.x = 0.2  # 前進速度 [m/s]
            twist.angular.z = 0.0 # 角速度 [rad/s]
        
        else:
            # collecting, stopping, initializing など、その他の状態では停止
            twist.linear.x = 0.0
            twist.angular.z = 0.0
        
        # 最終的な速度指令を配信
        self.cmd_vel_pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    movement_controller_node = MovementController()
    try:
        rclpy.spin(movement_controller_node)
    except KeyboardInterrupt:
        pass
    finally:
        movement_controller_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()