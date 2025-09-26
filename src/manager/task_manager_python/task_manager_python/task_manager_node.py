#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool, Empty, Float32
from collections import deque

class TaskManager(Node):
    def __init__(self):
        super().__init__('task_manager')

        # 状態の定義
        self.STATE_INITIALIZING = "initializing"
        self.STATE_SEARCHING = "searching"
        self.STATE_APPROACHING = "approaching"
        self.STATE_COLLECTING = "collecting"
        self.STATE_STOPPING = "stopping"
        
        self._state = ""
        self.target_queue = deque()
        
        # 状態遷移のためのタイマーを保持する変数
        self.transition_timer = None

        # Publisher / Subscriber
        self.state_pub = self.create_publisher(String, '/robot/state', 10)
        self.camera_angle_pub = self.create_publisher(Float32, '/cameraswingmotor/target_angle', 10)
        self.create_subscription(Empty, '/detected_objects_3d', self.detected_objects_callback, 10)
        self.create_subscription(Bool, '/hose/result', self.hose_result_callback, 10)

        # 状態を定期的に配信するタイマー
        self.create_timer(1.0, self.publish_state)

        self.get_logger().info("✅ Task Manager ノード (ROS2) が起動しました。")
        
        # 1. 初期化状態から開始
        self.set_state(self.STATE_INITIALIZING)

        # 初期化時にカメラの角度を設定
        self.initialize_camera()
        
        # 2. 2秒後に探索を開始
        self.transition_timer = self.create_timer(2.0, self.start_searching)

    def initialize_camera(self):
        self.get_logger().info("カメラの初期化を行います...")
        self.initial_angle_timer = self.create_timer(0.5, self._publish_initial_camera_angle)

    def _publish_initial_camera_angle(self):
        initial_angle = Float32()
        initial_angle.data = 20.0  # 初期角度を20度に設定
        self.camera_angle_pub.publish(initial_angle)
        self.get_logger().info(f"カメラの角度を {initial_angle.data} 度に設定しました。")

        if self.initial_angle_timer is not None:
            self.initial_angle_timer.cancel()

    def set_state(self, new_state):
        if self._state != new_state:
            self._state = new_state
            self.get_logger().info(f"====== 状態が [ {self._state.upper()} ] に遷移しました ======")
            self.publish_state()

            # 進行中の遷移タイマーがあればキャンセル
            if self.transition_timer is not None and not self.transition_timer.is_canceled():
                self.transition_timer.cancel()

            # 状態に応じた処理を開始
            if self._state == self.STATE_APPROACHING:
                self.get_logger().info("  -> ゴミに接近中...")
                # TODO: ナビゲーションノードへ目標を指示
                self.transition_timer = self.create_timer(3.0, self.on_approach_done)

            elif self._state == self.STATE_COLLECTING:
                self.get_logger().info("  -> ホースでゴミを回収中...")
                # TODO: ホース制御ノードへ回収開始を指示

            elif self._state == self.STATE_STOPPING:
                self.get_logger().info("  -> 一時停止中...")
                self.transition_timer = self.create_timer(1.0, self.on_stop_done)
                
    def publish_state(self):
        msg = String()
        msg.data = self._state
        self.state_pub.publish(msg)

    def start_searching(self):
        """初期化完了後、探索状態に移行するコールバック"""
        self.transition_timer.cancel()
        self.set_state(self.STATE_SEARCHING)

    def on_approach_done(self):
        """接近完了（シミュレーション）時に呼ばれるコールバック"""
        self.transition_timer.cancel()
        if self._state == self.STATE_APPROACHING:
            self.set_state(self.STATE_COLLECTING)

    def on_stop_done(self):
        """停止処理完了時に呼ばれるコールバック"""
        self.transition_timer.cancel()
        if self._state == self.STATE_STOPPING:
            if self.target_queue:
                self.target_queue.popleft()
            
            if self.target_queue:
                self.get_logger().info("次のゴミへ移動します。")
                self.set_state(self.STATE_APPROACHING)
            else:
                self.get_logger().info("全てのゴミを回収しました。探索を再開します。")
                self.set_state(self.STATE_SEARCHING)

    def detected_objects_callback(self, msg):
        if self._state == self.STATE_SEARCHING:
            self.get_logger().info("🗑️ ゴミを検出しました。タスクキューに追加します。")
            self.target_queue.append("dummy_target_1")
            self.set_state(self.STATE_APPROACHING)

    def hose_result_callback(self, msg):
        if self._state == self.STATE_COLLECTING:
            if msg.data:
                self.get_logger().info("👍 回収成功！")
            else:
                self.get_logger().info("😥 回収失敗...")
            self.set_state(self.STATE_STOPPING)

def main(args=None):
    rclpy.init(args=args)
    task_manager_node = TaskManager()
    try:
        rclpy.spin(task_manager_node)
    except KeyboardInterrupt:
        pass
    finally:
        task_manager_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()