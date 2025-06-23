#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from aruco_interfaces.msg._aruco_markers import ArucoMarkers  # 実際の定義に置き換えてください
import csv
import os
from rclpy.time import Time

class CSVLoggerNode(Node):
    def __init__(self):
        super().__init__('angle_arucopose_csv')

        # 保存ファイルパス
        self.filepath = os.path.expanduser("~/pickup_ws/angle_arucopose_csv/aruco_motor_log.csv")
        self.init_csv()

        # 最新データ
        self.latest_angles = None

        # Subscribe
        self.sub_angle = self.create_subscription(
            Float32MultiArray,
            '/motor_current_angles',
            self.angle_callback,
            10
        )

        self.sub_marker = self.create_subscription(
            ArucoMarkers,
            '/aruco/markers',
            self.marker_callback,
            10
        )

    def init_csv(self):
        os.makedirs(os.path.dirname(self.filepath), exist_ok=True)
        if not os.path.exists(self.filepath):
            with open(self.filepath, mode='w', newline='') as f:
                writer = csv.writer(f)
                header = ['timestamp'] + [f'motor{i+1}' for i in range(9)] + [
                    'marker_id', 'x', 'y', 'z', 'qx', 'qy', 'qz', 'qw'
                ]
                writer.writerow(header)

    def angle_callback(self, msg):
        if len(msg.data) == 9:
            self.latest_angles = list(msg.data)
        else:
            self.get_logger().warn('Invalid motor angle length.')

    def marker_callback(self, msg):
        if self.latest_angles is None:
            self.get_logger().warn('Motor angles not yet received.')
            return

        timestamp = self.get_clock().now().to_msg()
        unix_time = Time.from_msg(timestamp).nanoseconds * 1e-9

        for marker_id, pose in zip(msg.marker_ids, msg.poses):
            row = [unix_time] + self.latest_angles + [
                marker_id,
                pose.position.x,
                pose.position.y,
                pose.position.z,
                pose.orientation.x,
                pose.orientation.y,
                pose.orientation.z,
                pose.orientation.w,
            ]
            with open(self.filepath, mode='a', newline='') as f:
                writer = csv.writer(f)
                writer.writerow(row)

        self.get_logger().info(f"Logged {len(msg.marker_ids)} markers with motor angles.")

def main(args=None):
    rclpy.init(args=args)
    node = CSVLoggerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
