#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from aruco_interfaces.msg import ArucoMarkers
import csv
import os
from builtin_interfaces.msg import Time

class ArucoLogger(Node):
    def __init__(self):
        super().__init__('aruco_pose_csv')

        self.declare_parameter("output_csv", "aruco_log.csv")
        self.csv_path = self.get_parameter("output_csv").get_parameter_value().string_value

        self.subscription = self.create_subscription(
            ArucoMarkers,
            '/aruco/markers',
            self.marker_callback,
            10
        )

        # CSV初期化（ヘッダー書き込み）
        if not os.path.exists(self.csv_path):
            with open(self.csv_path, mode='w', newline='') as file:
                writer = csv.writer(file)
                writer.writerow(['timestamp', 'id', 'x', 'y', 'z', 'qx', 'qy', 'qz', 'qw'])

    def marker_callback(self, msg: ArucoMarkers):
        timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

        for i, pose in enumerate(msg.poses):
            marker_id = msg.marker_ids[i]
            pos = pose.position
            ori = pose.orientation

            row = [timestamp, marker_id, pos.x, pos.y, pos.z, ori.x, ori.y, ori.z, ori.w]
            with open(self.csv_path, mode='a', newline='') as file:
                writer = csv.writer(file)
                writer.writerow(row)

        self.get_logger().info(f"Logged {len(msg.poses)} marker(s)")

def main(args=None):
    rclpy.init(args=args)
    node = ArucoLogger()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
