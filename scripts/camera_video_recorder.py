#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
from datetime import datetime

import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class ImageVideoRecorder(Node):
    def __init__(self):
        super().__init__("image_video_recorder")

        self.declare_parameter("image_topic", "/camera/camera/color/image_raw")
        self.declare_parameter("output_dir", "/home/matsunaga-h/pickup_ws/videos")
        self.declare_parameter("fps", 30)
        self.declare_parameter("codec", "mp4v")  # "mp4v", "MJPG", "XVID" など
        self.declare_parameter("window_name", "ROS Image Recording (press 'q' to quit)")

        self.topic = self.get_parameter("image_topic").get_parameter_value().string_value
        self.output_dir = self.get_parameter("output_dir").get_parameter_value().string_value
        self.fps = self.get_parameter("fps").get_parameter_value().integer_value
        self.codec = self.get_parameter("codec").get_parameter_value().string_value
        self.window_name = self.get_parameter("window_name").get_parameter_value().string_value

        os.makedirs(self.output_dir, exist_ok=True)

        self.bridge = CvBridge()
        self.latest_bgr = None
        self.video_writer = None
        self.recording = False
        self.frame_width = None
        self.frame_height = None

        self.sub = self.create_subscription(Image, self.topic, self.cb_image, 10)

        # 表示 & キー入力処理用タイマ（30Hz）
        self.timer = self.create_timer(1.0 / 30.0, self.on_timer)

        cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)

        self.get_logger().info(f"Subscribing: {self.topic}")
        self.get_logger().info(f"Output dir : {self.output_dir}")
        self.get_logger().info(f"FPS: {self.fps}, Codec: {self.codec}")
        self.get_logger().info("Press 'r' to start/stop recording, 'q' to quit.")

    def cb_image(self, msg: Image):
        try:
            # color/image_raw は通常 bgr8 で来ることが多い
            self.latest_bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

            # 初回はframe sizeとvideo writerを初期化
            if self.frame_width is None:
                h, w = self.latest_bgr.shape[:2]
                self.frame_width = w
                self.frame_height = h

            # 録画中の場合はフレームを書き込む
            if self.recording and self.video_writer is not None:
                self.video_writer.write(self.latest_bgr)

        except Exception as e:
            self.get_logger().error(f"cv_bridge conversion failed: {e}")

    def start_recording(self):
        if self.latest_bgr is None:
            self.get_logger().warn("No image received yet.")
            return

        if self.recording:
            self.get_logger().warn("Already recording.")
            return

        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        video_path = os.path.join(self.output_dir, f"video_{ts}.mp4")

        try:
            fourcc = cv2.VideoWriter_fourcc(*self.codec)
            self.video_writer = cv2.VideoWriter(
                video_path, fourcc, self.fps, (self.frame_width, self.frame_height)
            )

            if not self.video_writer.isOpened():
                raise RuntimeError("Failed to open VideoWriter")

            self.recording = True
            self.get_logger().info(f"Started recording: {video_path}")

        except Exception as e:
            self.get_logger().error(f"Failed to start recording: {e}")
            if self.video_writer is not None:
                self.video_writer.release()
                self.video_writer = None

    def stop_recording(self):
        if not self.recording:
            self.get_logger().warn("Not recording.")
            return

        try:
            if self.video_writer is not None:
                self.video_writer.release()
                self.video_writer = None

            self.recording = False
            self.get_logger().info("Stopped recording.")

        except Exception as e:
            self.get_logger().error(f"Failed to stop recording: {e}")

    def on_timer(self):
        if self.latest_bgr is not None:
            # 録画中の場合は表示にテキストを追加
            display_frame = self.latest_bgr.copy()
            if self.recording:
                cv2.putText(
                    display_frame,
                    "REC",
                    (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    1.0,
                    (0, 0, 255),
                    2,
                )

            cv2.imshow(self.window_name, display_frame)

        key = cv2.waitKey(1) & 0xFF
        if key == ord("r"):
            if self.recording:
                self.stop_recording()
            else:
                self.start_recording()
        elif key == ord("q"):
            self.stop_recording()
            rclpy.shutdown()

    def destroy_node(self):
        try:
            self.stop_recording()
        except Exception:
            pass

        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ImageVideoRecorder()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
