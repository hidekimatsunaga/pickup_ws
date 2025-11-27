#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import cv2
import numpy as np


class DepthSaver(Node):
    def __init__(self):
        super().__init__("depth_saver")

        # ---- params ----
        self.topic = self.declare_parameter(
            "depth_topic",
            "/camera/camera/aligned_depth_to_color/image_raw"
        ).value

        self.out_dir = Path(self.declare_parameter(
            "out_dir",
            str(Path.home() / "depth_images")
        ).value)

        self.save_every_n = int(self.declare_parameter(
            "save_every_n_frames",
            1
        ).value)

        self.save_colormap = bool(self.declare_parameter(
            "save_colormap",
            True
        ).value)

        # colormap range (for visualization only)
        self.vis_min_mm = float(self.declare_parameter(
            "vis_min_mm", 200.0
        ).value)
        self.vis_max_mm = float(self.declare_parameter(
            "vis_max_mm", 2000.0
        ).value)

        self.out_dir.mkdir(parents=True, exist_ok=True)
        self.bridge = CvBridge()
        self.frame_count = 0

        self.sub = self.create_subscription(
            Image,
            self.topic,
            self.cb,
            10
        )

        self.get_logger().info(f"Subscribe: {self.topic}")
        self.get_logger().info(f"Output dir: {self.out_dir}")

    def cb(self, msg: Image):
        self.frame_count += 1
        if (self.frame_count % self.save_every_n) != 0:
            return

        # ---- ROS Image -> numpy ----
        try:
            # passthroughで元のエンコーディング維持
            depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        except Exception as e:
            self.get_logger().error(f"cv_bridge failed: {e}")
            return

        # depth shape: (H, W), dtype uint16 or float32
        if depth is None or depth.size == 0:
            return

        # ---- convert float(m) -> uint16(mm) if needed ----
        if depth.dtype == np.float32 or depth.dtype == np.float64:
            # meters -> millimeters
            depth_mm = (depth * 1000.0).astype(np.uint16)
        elif depth.dtype == np.uint16:
            depth_mm = depth
        else:
            self.get_logger().warn(f"Unexpected dtype: {depth.dtype}, try uint16 cast")
            depth_mm = depth.astype(np.uint16)

        # ---- filename ----
        stamp = msg.header.stamp
        t_ns = int(stamp.sec * 1e9 + stamp.nanosec)
        base = self.out_dir / f"depth_{t_ns}"

        # ---- save raw depth (16bit PNG) ----
        raw_path = str(base) + ".png"
        # cv2.imwrite handles 16-bit PNG if dtype=uint16
        ok = cv2.imwrite(raw_path, depth_mm)
        if not ok:
            self.get_logger().error(f"Failed to save: {raw_path}")
            return

        # ---- optional: save colored visualization ----
        if self.save_colormap:
            vis = self.make_vis(depth_mm,
                                vmin=self.vis_min_mm,
                                vmax=self.vis_max_mm)
            vis_path = str(base) + "_vis.png"
            cv2.imwrite(vis_path, vis)

        self.get_logger().info(f"Saved depth: {raw_path}")

    @staticmethod
    def make_vis(depth_mm: np.ndarray, vmin=200.0, vmax=2000.0):
        """depth(mm) -> colorized image for quick check"""
        depth_clipped = np.clip(depth_mm.astype(np.float32), vmin, vmax)
        norm = (depth_clipped - vmin) / (vmax - vmin + 1e-6)  # 0..1
        norm_u8 = (norm * 255.0).astype(np.uint8)
        vis = cv2.applyColorMap(norm_u8, cv2.COLORMAP_JET)
        return vis


def main():
    rclpy.init()
    node = DepthSaver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
