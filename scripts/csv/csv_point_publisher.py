#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped, Point
from visualization_msgs.msg import Marker
import pandas as pd
import numpy as np

class Nearest3Highlighter(Node):
    def __init__(self):
        super().__init__('nearest3_highlighter')

        # ===== 設定 =====
        self.frame_id = "camera_color_optical_frame"
        csv_path = "/home/matsunaga-h/pickup_ws/angle_arucopose_csv_cleaned/aruco_motor_log_1108_193312_cleaned.csv"
        self.base_scale = 0.01   # 全点のサイズ
        self.near_scale = 0.02   # 近傍3点のサイズ（強調）
        # =================

        self.get_logger().info("=== Nearest3Highlighter started ===")
        self.get_logger().info(f"frame_id: {self.frame_id}")
        self.get_logger().info(f"csv_path: {csv_path}")

        # CSV読み込み
        df = pd.read_csv(csv_path, header=None, skiprows=1)
        pos = df.iloc[:, 12:15].astype(float).to_numpy()  # Nx3
        self.pos_np = pos  # numpyで距離計算用
        self.all_points = []
        for x, y, z in pos:
            p = Point()
            p.x, p.y, p.z = float(x), float(y), float(z)
            self.all_points.append(p)

        self.get_logger().info(f"Loaded {len(self.all_points)} CSV points ✅")

        # publishers
        self.pub_all  = self.create_publisher(Marker, "/csv_points_marker", 10)
        self.pub_near = self.create_publisher(Marker, "/nearest3_marker", 10)

        # subscriber
        self.sub = self.create_subscription(
            PointStamped,
            "/detected_depth_points",
            self.cb_detected,
            10
        )

        # 最初に全点を1回出す（RVizでいつでも見えるように）
        self.publish_all_points()

    def publish_all_points(self):
        now = self.get_clock().now().to_msg()

        m = Marker()
        m.header.frame_id = self.frame_id
        m.header.stamp = now
        m.ns = "csv_points"
        m.id = 0
        m.type = Marker.SPHERE_LIST
        m.action = Marker.ADD

        m.scale.x = self.base_scale
        m.scale.y = self.base_scale
        m.scale.z = self.base_scale

        # 全点：青
        m.color.r = 0.1
        m.color.g = 0.3
        m.color.b = 1.0
        m.color.a = 1.0

        m.points = self.all_points
        self.pub_all.publish(m)
        self.get_logger().info("Published all CSV points (blue)")

    def cb_detected(self, msg: PointStamped):
        x = msg.point.x
        y = msg.point.y
        z = msg.point.z
        fid = msg.header.frame_id

        # フレーム違いの安全チェック（今回は同じ想定）
        if fid != self.frame_id:
            self.get_logger().warn(
                f"Frame mismatch: detected={fid}, csv={self.frame_id}. "
                "TF変換しないと正しく最近傍が取れない可能性あり。"
            )

        # 距離計算 → 近い順に3つ
        target = np.array([x, y, z], dtype=float)
        dists = np.linalg.norm(self.pos_np - target, axis=1)
        nearest_idx = np.argsort(dists)[:3]

        self.get_logger().info(
            f"Detected depth point: ({x:.3f}, {y:.3f}, {z:.3f}) "
            f"-> nearest idx {nearest_idx.tolist()} "
            f"dist {dists[nearest_idx].round(4).tolist()}"
        )

        # 近傍3点のMarker（赤で強調）
        now = self.get_clock().now().to_msg()
        near = Marker()
        near.header.frame_id = self.frame_id
        near.header.stamp = now
        near.ns = "nearest3"
        near.id = 0
        near.type = Marker.SPHERE_LIST
        near.action = Marker.ADD

        near.scale.x = self.near_scale
        near.scale.y = self.near_scale
        near.scale.z = self.near_scale

        # 近傍3点：赤
        near.color.r = 1.0
        near.color.g = 0.1
        near.color.b = 0.1
        near.color.a = 1.0

        near.points = [self.all_points[i] for i in nearest_idx]
        self.pub_near.publish(near)

def main(args=None):
    rclpy.init(args=args)
    node = Nearest3Highlighter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
