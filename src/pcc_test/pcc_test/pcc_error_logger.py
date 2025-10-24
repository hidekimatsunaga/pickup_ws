#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import math
import numpy as np
import rclpy
from rclpy.node import Node
import csv
import os
from datetime import datetime

# msgs
from geometry_msgs.msg import PointStamped, Point, Pose
from aruco_interfaces.msg import ArucoMarkers  # ids:int32[], poses:Pose[]

# ===== ユーティリティ (前のPCCMoveNodeからコピー) =====
def rot_from_quat(w, x, y, z):
    return np.array([
        [1-2*(y*y+z*z), 2*(x*y - z*w), 2*(x*z + y*w)],
        [2*(x*y + z*w), 1-2*(x*x+z*z), 2*(y*z - x*w)],
        [2*(x*z - y*w), 2*(y*z + x*w), 1-2*(x*x+y*y)]
    ], float)

def T_from_Rt(R, t):
    T = np.eye(4); T[:3,:3] = R; T[:3,3] = t
    return T

def T_inv(T):
    R = T[:3,:3]; t = T[:3,3]
    Ti = np.eye(4); Ti[:3,:3] = R.T; Ti[:3,3] = -R.T @ t
    return Ti

# ===== ノード =====
class PCCErrorLogger(Node):
    def __init__(self):
        super().__init__('pcc_error_logger')

        # --- パラメータ ---
        self.declare_parameter('base_marker_id', 4)         # 基準マーカ (id4)
        self.declare_parameter('tip_marker_id', 0)          # 先端マーカ (id0)
        self.declare_parameter('csv_filename', 'pcc_error_log.csv')
        self.declare_parameter('camera_frame', 'camera_color_optical_frame')

        # 取得
        self.base_id = int(self.get_parameter('base_marker_id').value)
        self.tip_id = int(self.get_parameter('tip_marker_id').value)
        self.csv_filename = self.get_parameter('csv_filename').value
        self.cam_frame = self.get_parameter('camera_frame').value.lower()
        
        # 状態
        self.T_BC = None                  # B←C (ベース←カメラ)
        self.last_tip_pB = None           # 先端(id0)のベース座標系での最新位置
        self.last_target_pB = None        # 目標のベース座標系での最新位置
        self.tip_detected_once = False
        self.base_detected_once = False

        # CSVファイル準備
        self.csv_file = None
        self.csv_writer = None
        try:
            # 実行ディレクトリにCSVを作成
            filepath = os.path.join(os.getcwd(), self.csv_filename)
            self.csv_file = open(filepath, 'w', newline='')
            self.csv_writer = csv.writer(self.csv_file)
            # ヘッダ書き込み
            self.csv_writer.writerow([
                'timestamp_sec', 
                'target_x', 'target_y', 'target_z', 
                'actual_x', 'actual_y', 'actual_z',
                'error_dist', 'error_x', 'error_y', 'error_z'
            ])
            self.get_logger().info(f"Logging errors to: {filepath}")
        except Exception as e:
            self.get_logger().error(f"Failed to open CSV file: {e}")
            rclpy.shutdown()
            return

        # I/O
        self.sub_mark = self.create_subscription(
            ArucoMarkers, '/aruco/markers', self.cb_markers, 10)
        self.sub_target = self.create_subscription(
            PointStamped, '/pcc_target', self.cb_target_and_log, 10) # ★目標受信とログを兼ねる

        # シャットダウン時の処理
        self.context.on_shutdown(self._on_shutdown)


    def _on_shutdown(self):
        """ノード終了時にCSVファイルを閉じる"""
        if self.csv_file:
            self.csv_file.close()
            self.get_logger().info("CSV file closed.")

    def cb_markers(self, msg: ArucoMarkers):
        """
        マーカ情報を受信し、T_BC (ベース変換) と last_tip_pB (先端位置) を更新する
        """
        if len(msg.marker_ids) == 0:
            return

        ids = list(msg.marker_ids)
        poses_C = msg.poses # カメラ座標系(C)でのポーズリスト
        
        T_CB_found = False
        pC_tip_found = False
        
        pC_tip = None # 先端(id0)のカメラ座標系での位置

        try:
            # 1. ベース(id4)を探して T_BC を更新
            if self.base_id in ids:
                i = ids.index(self.base_id)
                base_pose: Pose = poses_C[i]
                t = np.array([base_pose.position.x, base_pose.position.y, base_pose.position.z], float)
                q = base_pose.orientation
                R = rot_from_quat(q.w, q.x, q.y, q.z)
                T_CB = T_from_Rt(R, t)   # C←B
                self.T_BC = T_inv(T_CB)  # B←C
                T_CB_found = True
                if not self.base_detected_once:
                    self.get_logger().info(f"Base (id={self.base_id}) detected. T_BC ready.")
                    self.base_detected_once = True

            # 2. 先端(id0)を探して pC_tip を取得
            if self.tip_id in ids:
                j = ids.index(self.tip_id)
                tip_pose: Pose = poses_C[j]
                pC_tip = np.array([
                    tip_pose.position.x, 
                    tip_pose.position.y, 
                    tip_pose.position.z
                ], float)
                pC_tip_found = True
                if not self.tip_detected_once:
                    self.get_logger().info(f"Tip (id={self.tip_id}) detected.")
                    self.tip_detected_once = True
            
            # 3. 両方揃ったら、先端のベース座標系(pB)を計算
            if pC_tip_found and self.T_BC is not None:
                pC_tip_homo = np.array([pC_tip[0], pC_tip[1], pC_tip[2], 1.0], float)
                pB_tip = (self.T_BC @ pC_tip_homo)[:3]
                self.last_tip_pB = pB_tip # これが最新の「実績値」
                
        except Exception as e:
            self.get_logger().warn(f'/aruco/markers parse error: {e}')

    def cb_target_and_log(self, msg: PointStamped):
        """
        目標値(/pcc_target)を受信したタイミングで、
        最新の実績値(last_tip_pB)との誤差を計算し、CSVに書き込む
        """
        
        # ---- 1. 目標値 (Target) を取得 ----
        frame = (msg.header.frame_id or '').lower()
        
        if frame.startswith('base'):
            # frame_id が 'base' (id4基準) だった場合
            p_target = np.array([msg.point.x, msg.point.y, msg.point.z], float)
            self.last_target_pB = p_target
        
        elif frame == self.cam_frame or frame.startswith('camera'):
            # frame_id が 'camera' だった場合 (T_BC が必要)
            if self.T_BC is None:
                self.get_logger().warn("Received target in camera frame, but Base (id4) not detected yet. Skipping log.")
                return
            pC = np.array([msg.point.x, msg.point.y, msg.point.z, 1.0], float)
            p_target = (self.T_BC @ pC)[:3]
            self.last_target_pB = p_target
        
        else:
            self.get_logger().warn(f"Unknown target frame_id '{msg.header.frame_id}'. Assuming 'base' frame.")
            p_target = np.array([msg.point.x, msg.point.y, msg.point.z], float)
            self.last_target_pB = p_target

        # ---- 2. 実績値 (Actual) を取得 ----
        if self.last_tip_pB is None:
            self.get_logger().warn(f"Received target, but Tip (id={self.tip_id}) not detected yet. Skipping log.")
            return
            
        p_actual = self.last_tip_pB
        
        # ---- 3. 誤差 (Error) を計算 ----
        err_vec = p_target - p_actual
        err_dist = np.linalg.norm(err_vec)
        
        # ---- 4. CSVに書き込み ----
        if self.csv_writer:
            try:
                # タイムスタンプ (秒)
                ts = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
                
                row_data = [
                    f"{ts:.6f}",
                    f"{p_target[0]:.6f}", f"{p_target[1]:.6f}", f"{p_target[2]:.6f}",
                    f"{p_actual[0]:.6f}", f"{p_actual[1]:.6f}", f"{p_actual[2]:.6f}",
                    f"{err_dist:.6f}",
                    f"{err_vec[0]:.6f}", f"{err_vec[1]:.6f}", f"{err_vec[2]:.6f}"
                ]
                self.csv_writer.writerow(row_data)
                # self.csv_file.flush() # 1行ごと保存したい場合はコメント解除
                
                self.get_logger().info(f"Logged: Target=({p_target[0]:.3f},{p_target[2]:.3f}), Actual=({p_actual[0]:.3f},{p_actual[2]:.3f}), Err={err_dist:.4f}")
                
            except Exception as e:
                self.get_logger().error(f"Failed to write to CSV: {e}")


def main():
    rclpy.init()
    node = PCCErrorLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # ノードの破棄（これにより _on_shutdown が呼ばれる）
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()