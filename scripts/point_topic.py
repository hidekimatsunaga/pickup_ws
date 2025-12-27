#!/usr/bin/env python3
import numpy as np
import cv2

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import CameraInfo, Image
from geometry_msgs.msg import PointStamped
from cv_bridge import CvBridge


class ArucoImagePointOverlay(Node):
    def __init__(self):
        super().__init__('aruco_image_point_overlay')

        # ===== 固定トピック =====
        self.CAM_INFO_TOPIC = "/camera/camera/color/camera_info"
        self.IMAGE_IN_TOPIC = "/aruco/image"
        self.POINT_TOPIC    = "/detected_depth_points"

        self.IMAGE_OUT_TOPIC = "/aruco/image_with_point"

        # ===== 描画設定 =====
        self.radius_px = 6      # 黒丸の半径
        self.thickness = -1     # 塗りつぶし
        self.draw_when_outside = False  # 画角外なら描かない

        # ===== 状態 =====
        self.K = None
        self.last_uv_int = None  # (u,v) int

        self.bridge = CvBridge()

        self.sub_info = self.create_subscription(CameraInfo, self.CAM_INFO_TOPIC, self.on_cam_info, 10)
        self.sub_pt   = self.create_subscription(PointStamped, self.POINT_TOPIC, self.on_point, 10)
        self.sub_img  = self.create_subscription(Image, self.IMAGE_IN_TOPIC, self.on_image, 10)

        self.pub_img  = self.create_publisher(Image, self.IMAGE_OUT_TOPIC, 10)

        self.get_logger().info(f"Subscribe CameraInfo: {self.CAM_INFO_TOPIC}")
        self.get_logger().info(f"Subscribe Image:      {self.IMAGE_IN_TOPIC}")
        self.get_logger().info(f"Subscribe Point:      {self.POINT_TOPIC}")
        self.get_logger().info(f"Publish Overlay:      {self.IMAGE_OUT_TOPIC}")

    def on_cam_info(self, msg: CameraInfo):
        self.K = np.array(msg.k, dtype=np.float64).reshape(3, 3)
        fx, fy = self.K[0, 0], self.K[1, 1]
        cx, cy = self.K[0, 2], self.K[1, 2]
        self.get_logger().info(
            f"CameraInfo updated: fx={fx:.3f}, fy={fy:.3f}, cx={cx:.3f}, cy={cy:.3f}"
        )

    def project_to_pixel(self, X: float, Y: float, Z: float):
        if self.K is None or Z <= 0.0:
            return None
        fx = self.K[0, 0]
        fy = self.K[1, 1]
        cx = self.K[0, 2]
        cy = self.K[1, 2]
        u = fx * (X / Z) + cx
        v = fy * (Y / Z) + cy
        return u, v

    def on_point(self, msg: PointStamped):
        uv = self.project_to_pixel(float(msg.point.x), float(msg.point.y), float(msg.point.z))
        if uv is None:
            self.last_uv_int = None
            return

        u, v = uv
        ui = int(round(u))
        vi = int(round(v))
        self.last_uv_int = (ui, vi)

    def on_image(self, msg: Image):
        # /aruco/image が bgr8 とは限らないので安全に変換
        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().error(f"imgmsg_to_cv2 failed: {e}")
            return

        H, W = cv_img.shape[:2]

        # 点が来ていれば描画
        if self.last_uv_int is not None:
            ui, vi = self.last_uv_int
            in_range = (0 <= ui < W) and (0 <= vi < H)

            if in_range or self.draw_when_outside:
                cv2.circle(cv_img, (ui, vi), self.radius_px, (0, 0, 0), self.thickness)

        out = self.bridge.cv2_to_imgmsg(cv_img, encoding="bgr8")
        out.header = msg.header  # 元画像のheader維持
        self.pub_img.publish(out)


def main():
    rclpy.init()
    node = ArucoImagePointOverlay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
