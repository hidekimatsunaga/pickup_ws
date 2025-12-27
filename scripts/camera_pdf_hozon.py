#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import tempfile
from datetime import datetime

import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from reportlab.pdfgen import canvas as pdf_canvas


class ImagePdfSaver(Node):
    def __init__(self):
        super().__init__("image_pdf_saver")

        self.declare_parameter("image_topic", "/camera/camera/color/image_raw")
        self.declare_parameter("output_dir", os.getcwd())
        self.declare_parameter("window_name", "ROS Image (press 's' to save PDF, 'q' to quit)")

        self.topic = self.get_parameter("image_topic").get_parameter_value().string_value
        self.output_dir = self.get_parameter("output_dir").get_parameter_value().string_value
        self.window_name = self.get_parameter("window_name").get_parameter_value().string_value

        os.makedirs(self.output_dir, exist_ok=True)

        self.bridge = CvBridge()
        self.latest_bgr = None

        self.sub = self.create_subscription(Image, self.topic, self.cb_image, 10)

        # 表示 & キー入力処理用タイマ（30Hz）
        self.timer = self.create_timer(1.0 / 30.0, self.on_timer)

        cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)

        self.get_logger().info(f"Subscribing: {self.topic}")
        self.get_logger().info(f"Output dir : {self.output_dir}")
        self.get_logger().info("Press 's' to save current frame as PDF, 'q' to quit.")

    def cb_image(self, msg: Image):
        try:
            # color/image_raw は通常 bgr8 で来ることが多い
            self.latest_bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().error(f"cv_bridge conversion failed: {e}")

    def save_current_frame_as_pdf(self):
        if self.latest_bgr is None:
            self.get_logger().warn("No image received yet.")
            return

        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        pdf_path = os.path.join(self.output_dir, f"frame_{ts}.pdf")

        h, w = self.latest_bgr.shape[:2]

        # いったんPNGにしてからPDFへ貼り込む（確実）
        tmp_png = None
        try:
            fd, tmp_png = tempfile.mkstemp(prefix="ros_frame_", suffix=".png")
            os.close(fd)

            ok = cv2.imwrite(tmp_png, self.latest_bgr)
            if not ok:
                raise RuntimeError("cv2.imwrite failed")

            c = pdf_canvas.Canvas(pdf_path, pagesize=(w, h))
            # 左下原点なので (0,0) に画像をぴったり貼る
            c.drawImage(tmp_png, 0, 0, width=w, height=h)
            c.showPage()
            c.save()

            self.get_logger().info(f"Saved PDF: {pdf_path}")

        except Exception as e:
            self.get_logger().error(f"Failed to save PDF: {e}")
        finally:
            if tmp_png and os.path.exists(tmp_png):
                try:
                    os.remove(tmp_png)
                except Exception:
                    pass

    def on_timer(self):
        if self.latest_bgr is not None:
            cv2.imshow(self.window_name, self.latest_bgr)

        key = cv2.waitKey(1) & 0xFF
        if key == ord("s"):
            self.save_current_frame_as_pdf()
        elif key == ord("q"):
            rclpy.shutdown()

    def destroy_node(self):
        try:
            cv2.destroyAllWindows()
        except Exception:
            pass
        super().destroy_node()


def main():
    rclpy.init()
    node = ImagePdfSaver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
