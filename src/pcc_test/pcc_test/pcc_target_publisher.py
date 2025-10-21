#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from builtin_interfaces.msg import Time

class PCCTargetPublisher(Node):
    """
    目標先端位置を /pcc_target へ送るユーティリティノード。

    モード:
      - mode='once'        : 起動時に1回だけ publish
      - mode='continuous'  : hz で周期 publish
      - mode='from_click'  : RViz の /clicked_point を受け取り中継 publish

    フレーム:
      - frame_id: 'base' or 'camera_color_optical_frame' など
      - from_click の場合、force_frame_id=True なら header.frame_id を上書き
    """
    def __init__(self):
        super().__init__('pcc_target_publisher')

        # ---- parameters ----
        self.declare_parameter('mode', 'once')  # 'once' | 'continuous' | 'from_click'
        self.declare_parameter('frame_id', 'base')
        self.declare_parameter('x', 0.20)   # m
        self.declare_parameter('y', 0.00)   # m
        self.declare_parameter('z', 0.30)   # m
        self.declare_parameter('hz', 10.0)  # mode=continuous の送信周期
        self.declare_parameter('force_frame_id', True)  # from_click の frame_id 上書き

        self.mode = self.get_parameter('mode').value.lower()
        self.frame_id = self.get_parameter('frame_id').value
        self.target = [float(self.get_parameter('x').value),
                       float(self.get_parameter('y').value),
                       float(self.get_parameter('z').value)]
        self.hz = float(self.get_parameter('hz').value)
        self.force_frame = bool(self.get_parameter('force_frame_id').value)

        self.pub = self.create_publisher(PointStamped, '/pcc_target', 10)

        if self.mode == 'from_click':
            # RViz: "Publish Point" ツールは /clicked_point に PointStamped を出す
            self.sub = self.create_subscription(PointStamped, '/clicked_point',
                                                self.cb_clicked_point, 10)
            self.get_logger().info(
                f"[from_click] forwarding /clicked_point -> /pcc_target "
                f"(force_frame_id={self.force_frame}, frame_id='{self.frame_id}')")
        elif self.mode == 'continuous':
            dt = max(1e-3, 1.0 / max(1e-6, self.hz))
            self.timer = self.create_timer(dt, self.on_timer)
            self.get_logger().info(
                f"[continuous] publishing {self.target} in frame '{self.frame_id}' at {self.hz} Hz")
        else:
            # once
            self.get_logger().info(
                f"[once] publishing {self.target} in frame '{self.frame_id}'")
            self.publish_target(self.frame_id, *self.target)

    # ---------- helpers ----------
    def now_stamp(self) -> Time:
        return self.get_clock().now().to_msg()

    def publish_target(self, frame_id: str, x: float, y: float, z: float):
        msg = PointStamped()
        msg.header.frame_id = frame_id
        msg.header.stamp = self.now_stamp()
        msg.point.x = float(x); msg.point.y = float(y); msg.point.z = float(z)
        self.pub.publish(msg)
        self.get_logger().info(f"-> /pcc_target [{frame_id}]: x={x:.3f}, y={y:.3f}, z={z:.3f}")

    # ---------- callbacks ----------
    def on_timer(self):
        self.publish_target(self.frame_id, *self.target)

    def cb_clicked_point(self, msg: PointStamped):
        # RViz側から来た座標（通常は RViz の Fixed Frame に依存）
        if self.force_frame:
            # 強制的に frame_id を上書き
            self.publish_target(self.frame_id, msg.point.x, msg.point.y, msg.point.z)
        else:
            # そのまま中継（frame_id は元のまま）
            out = PointStamped()
            out.header = msg.header
            out.point = msg.point
            self.pub.publish(out)
            self.get_logger().info(
                f"-> /pcc_target [passthrough '{out.header.frame_id}']: "
                f"x={out.point.x:.3f}, y={out.point.y:.3f}, z={out.point.z:.3f}")

def main():
    rclpy.init()
    rclpy.spin(PCCTargetPublisher())
    rclpy.shutdown()

if __name__ == '__main__':
    main()
