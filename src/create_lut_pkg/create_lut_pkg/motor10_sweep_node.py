#!/usr/bin/env python3
"""
aruco_motor_log_1213_203430.csv の各行に対して
motor10を段階的に変更して、ホース先端位置の変化をマッピング
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Int8MultiArray
from aruco_interfaces.msg._aruco_markers import ArucoMarkers
import pandas as pd
import numpy as np
import threading
import time
import csv
import os
from datetime import datetime


class Motor10SweepNode(Node):
    """
    各motor設定に対してmotor10をスイープし、
    ホース先端位置の変化を記録
    """
    
    def __init__(self):
        super().__init__('motor10_sweep_node')
        
        # パラメータ
        self.declare_parameter('csv_path', os.path.expanduser("~/pickup_ws/angle_arucopose_csv/aruco_motor_log_1213_203430.csv"))
        self.declare_parameter('marker_id', 0)
        self.declare_parameter('motor10_start', 100.0)
        self.declare_parameter('motor10_end', 500.0)
        self.declare_parameter('motor10_step', 20.0)
        self.declare_parameter('settle_time', 1.0)  # 各motor10設定で待つ時間
        
        self.csv_path = self.get_parameter('csv_path').value
        self.marker_id = self.get_parameter('marker_id').value
        self.motor10_start = self.get_parameter('motor10_start').value
        self.motor10_end = self.get_parameter('motor10_end').value
        self.motor10_step = self.get_parameter('motor10_step').value
        self.settle_time = self.get_parameter('settle_time').value
        
        # Pub/Sub
        self.pub_motor = self.create_publisher(Float32MultiArray, '/motor_angles', 10)
        
        self.sub_marker = self.create_subscription(
            ArucoMarkers, '/aruco/markers', self.marker_callback, 10
        )
        
        # データ保存
        self.output_dir = os.path.expanduser("~/pickup_ws/angle_arucopose_csv/")
        os.makedirs(self.output_dir, exist_ok=True)
        
        timestamp = datetime.now().strftime("%m%d_%H%M%S")
        self.output_csv = os.path.join(self.output_dir, f"motor10_sweep_{timestamp}.csv")
        
        # 状態
        self.latest_marker = None
        self.is_collecting = False
        self.current_motor10 = None
        self.current_base_motors = None
        
        # スレッド
        self.sweep_thread = None
        self.stop_flag = False
        
        self.get_logger().info(f"Motor10 Sweep Node started")
        self.get_logger().info(f"CSV input: {self.csv_path}")
        self.get_logger().info(f"Output CSV: {self.output_csv}")
        self.get_logger().info(f"Motor10 range: {self.motor10_start}° ~ {self.motor10_end}° (step: {self.motor10_step}°)")
    
    def marker_callback(self, msg: ArucoMarkers):
        """ArUcoマーカーを受信"""
        if len(msg.marker_ids) > 0:
            # marker_idに対応するマーカーを探す
            for mid, pose in zip(msg.marker_ids, msg.poses):
                if mid == self.marker_id:
                    self.latest_marker = {
                        'id': mid,
                        'x': pose.position.x,
                        'y': pose.position.y,
                        'z': pose.position.z,
                        'timestamp': self.get_clock().now().nanoseconds
                    }
                    break
    
    def publish_motor_angles(self, motors):
        """モーター角度を発行"""
        msg = Float32MultiArray()
        msg.data = [float(m) for m in motors]
        self.pub_motor.publish(msg)
    
    def sweep_motor10_for_row(self, row_data, row_index):
        """
        特定のモーター設定に対してmotor10をスイープ
        row_data: [motor1, motor2, ..., motor9]
        """
        
        results = []
        motor10_values = np.arange(self.motor10_start, self.motor10_end + self.motor10_step, self.motor10_step)
        
        self.get_logger().info(f"\n{'='*60}")
        self.get_logger().info(f"行 {row_index}: motor1-9 = {row_data}")
        self.get_logger().info(f"Motor10をスイープ中... ({len(motor10_values)}ポイント)")
        
        for motor10_val in motor10_values:
            if self.stop_flag:
                self.get_logger().warn("スイープを中止しました")
                return results
            
            # モーター指令（motor1-9 + motor10）
            motor_cmd = list(row_data) + [motor10_val]
            self.publish_motor_angles(motor_cmd)
            
            # 指令が反映されるのを待つ
            time.sleep(self.settle_time)
            
            # ArUcoマーカーが受け取られたか確認
            if self.latest_marker is None:
                self.get_logger().warn(f"  Motor10={motor10_val:.1f}°: マーカー未検出")
                continue
            
            marker = self.latest_marker
            result = {
                'row_index': row_index,
                'motor1': row_data[0],
                'motor2': row_data[1],
                'motor3': row_data[2],
                'motor4': row_data[3],
                'motor5': row_data[4],
                'motor6': row_data[5],
                'motor7': row_data[6],
                'motor8': row_data[7],
                'motor9': row_data[8],
                'motor10': motor10_val,
                'marker_id': marker['id'],
                'x': marker['x'],
                'y': marker['y'],
                'z': marker['z'],
            }
            results.append(result)
            
            self.get_logger().info(
                f"  Motor10={motor10_val:.1f}°: x={marker['x']:.4f}, y={marker['y']:.4f}, z={marker['z']:.4f}"
            )
        
        self.get_logger().info(f"行 {row_index} 完了: {len(results)}ポイント取得\n")
        return results
    
    def run_sweep(self):
        """メイン処理：各行に対してmotor10をスイープ"""
        
        # 入力CSVを読み込み
        if not os.path.exists(self.csv_path):
            self.get_logger().error(f"CSVが見つかりません: {self.csv_path}")
            return
        
        df = pd.read_csv(self.csv_path, header=None, skiprows=1)
        
        # motor1-9を抽出（1-10列目、インデックス0-8）
        motors_1_to_9 = df.iloc[:, 0:9].values
        
        self.get_logger().info(f"入力CSVから {len(motors_1_to_9)} 行のmotor設定を読み込み")
        
        # 出力CSVヘッダーを作成
        header = ['row_index'] + [f'motor{i}' for i in range(1, 10)] + [
            'motor10', 'marker_id', 'x', 'y', 'z'
        ]
        
        all_results = []
        
        # 各行に対してスイープ
        for idx, motors in enumerate(motors_1_to_9):
            if self.stop_flag:
                break
            
            results = self.sweep_motor10_for_row(motors, idx)
            all_results.extend(results)
        
        # 結果をCSVに保存
        if all_results:
            with open(self.output_csv, 'w', newline='') as f:
                writer = csv.DictWriter(f, fieldnames=header)
                writer.writeheader()
                writer.writerows(all_results)
            
            self.get_logger().info(f"\n{'='*60}")
            self.get_logger().info(f"結果を保存しました: {self.output_csv}")
            self.get_logger().info(f"合計 {len(all_results)} ポイント")
            self.get_logger().info(f"{'='*60}\n")
        else:
            self.get_logger().warn("結果がありません")
    
    def start_sweep(self):
        """スイープ処理をスレッドで開始"""
        self.sweep_thread = threading.Thread(target=self.run_sweep)
        self.sweep_thread.daemon = True
        self.sweep_thread.start()


def main(args=None):
    rclpy.init(args=args)
    node = Motor10SweepNode()
    
    # スイープを開始
    node.start_sweep()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("終了します...")
        node.stop_flag = True
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
