#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Float32
import csv
import os
import math
import sys
import select


class CameraSwingCalibRecorder(Node):
    def __init__(self):
        super().__init__('camera_swing_calib_recorder')

        # CSVの保存先（適宜変えてOK）
        csv_path = "/home/matsunaga-h/pickup_ws/src/object_chaser/csv/camera_swing_calib_yz.csv"
        os.makedirs(os.path.dirname(csv_path), exist_ok=True)
        self.csv_path = csv_path

        # 最新値の保持
        self.current_point = None        # /detected_depth_points (camera frame)
        self.current_angle_deg = None    # /cameraswingmotor/angle (deg)

        # サブスクライブ
        self.sub_point = self.create_subscription(
            PointStamped, "/detected_depth_points", self.point_callback, 10)
        self.sub_angle = self.create_subscription(
            Float32, "/cameraswingmotor/angle", self.angle_callback, 10)

        # CSVヘッダ（初回だけ）
        if not os.path.exists(self.csv_path):
            with open(self.csv_path, "w", newline="") as f:
                w = csv.writer(f)
                w.writerow(["time_sec", "y_cam", "z_cam", "yz_norm", "camera_angle_deg"])

        self.get_logger().info(
            f"[Manual calib] Started. CSV: {self.csv_path}\n"
            f"  ターミナルで 's' + Enter → サンプル保存\n"
            f"  'q' + Enter → 終了"
        )

    # ---- トピック更新コールバック ----
    def point_callback(self, msg: PointStamped):
        self.current_point = msg

    def angle_callback(self, msg: Float32):
        self.current_angle_deg = msg.data

    # ---- 実際に保存する処理 ----
    def save_sample(self):
        if self.current_point is None or self.current_angle_deg is None:
            self.get_logger().warn("まだ /detected_depth_points か /cameraswingmotor/angle が来てないので保存できません")
            return

        y = self.current_point.point.y
        z = self.current_point.point.z
        r = math.hypot(y, z)
        t = self.get_clock().now().nanoseconds * 1e-9

        with open(self.csv_path, "a", newline="") as f:
            w = csv.writer(f)
            w.writerow([f"{t:.3f}", f"{y:.4f}", f"{z:.4f}", f"{r:.4f}", f"{self.current_angle_deg:.2f}"])

        self.get_logger().info(
            f"✅ Saved sample: y={y:.3f}, z={z:.3f}, |yz|={r:.3f}, angle={self.current_angle_deg:.2f} deg"
        )


def main(args=None):
    rclpy.init(args=args)
    node = CameraSwingCalibRecorder()

    try:
        # 自前ループ：ROSを回しつつキーボードを見る
        while rclpy.ok():
            # ROS コールバック処理（タイムアウトつき1ステップ）
            rclpy.spin_once(node, timeout_sec=0.1)

            # 標準入力に何か来てるかチェック（ノンブロッキング）
            readable, _, _ = select.select([sys.stdin], [], [], 0.0)
            if sys.stdin in readable:
                line = sys.stdin.readline().strip()
                if line == "s":
                    node.save_sample()
                elif line == "q":
                    node.get_logger().info("終了コマンド 'q' を受け取ったので、ノードを終了します。")
                    break
                # それ以外は無視
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
