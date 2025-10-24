#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import csv, os
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped, Pose
from aruco_interfaces.msg import ArucoMarkers
from rclpy.time import Time

# 座標変換用の数学関数
def rot_from_quat(w,x,y,z):
    return np.array([
        [1-2*(y*y+z*z), 2*(x*y - z*w), 2*(x*z + y*w)],
        [2*(x*y + z*w), 1-2*(x*x+z*z), 2*(y*z - x*w)],
        [2*(x*z - y*w), 2*(y*z + x*w), 1-2*(x*x+y*y)]
    ], float)

def T_from_Rt(R,t):
    T = np.eye(4); T[:3,:3]=R; T[:3,3]=t; return T

def T_inv(T):
    R=T[:3,:3]; t=T[:3,3]
    Ti=np.eye(4); Ti[:3,:3]=R.T; Ti[:3,3] = -R.T @ t
    return Ti

class ErrorLoggerNode(Node):
    """
    /hose/goal_point を受信するたびに、
    その時点での Aruco(ID 0) の ID 4 基準の座標を計算し、
    目標値との誤差をCSVに記録するノード。
    """
    def __init__(self):
        super().__init__('pcc_error_logger')

        # ---- Parameters ----
        self.declare_parameter('csv_path', '~/pickup_ws/src/pcc_test/error_log.csv')
        self.declare_parameter('base_marker_id', 4)
        self.declare_parameter('tip_marker_id', 0)

        # ---- Get Parameters ----
        self.csv_path = os.path.expanduser(self.get_parameter('csv_path').value)
        self.base_id = int(self.get_parameter('base_marker_id').value)
        self.tip_id = int(self.get_parameter('tip_marker_id').value)

        # ---- State (最新の状態を保持) ----
        self.T_BC = None # Base <- Camera の変換行列
        self.tip_pose_C = None # Tip (ID 0) の Camera 座標系での Pose
        self.last_aruco_stamp = None # Aruco を最後に受信した時刻
        self._last_log_times = {} # ★ スロットリング用

        # ---- I/O ----
        # Arucoマーカー情報（状態更新用）
        self.sub_markers = self.create_subscription(
            ArucoMarkers, '/aruco/markers', self.cb_markers, 10)
        
        # Goal（トリガー用）
        self.sub_goal = self.create_subscription(
            PointStamped, '/hose/goal_point', self.cb_goal, 10)

        self._ensure_header()
        self.get_logger().info(f"ErrorLoggerNode ready. Logging to: {self.csv_path}")

    def _ensure_header(self):
        """CSVファイルが存在しない場合、ヘッダを書き込む"""
        if os.path.exists(self.csv_path) and os.path.getsize(self.csv_path) > 0:
            return
        try:
            with open(self.csv_path, 'w', newline='') as f:
                w = csv.writer(f)
                w.writerow([
                    'goal_stamp', 'aruco_stamp', 'goal_frame_id',
                    'goal_x_B', 'goal_y_B', 'goal_z_B',
                    'actual_x_B', 'actual_y_B', 'actual_z_B',
                    'error_m'
                ])
            self.get_logger().info(f"CSV header written to {self.csv_path}")
        except Exception as e:
            self.get_logger().error(f"Failed to write CSV header: {e}")

    # ★ 修正: スロットリング用関数を追加
    def _log_throttle(self, level: str, key: str, msg: str, period_sec: float = 2.0):
        """
        level: 'debug'|'info'|'warn'|'error'
        key:   同じメッセージ系列をまとめる識別子
        """
        now = self.get_clock().now().nanoseconds * 1e-9
        last = self._last_log_times.get(key, 0.0)
        if now - last < period_sec:
            return
        self._last_log_times[key] = now
        logger = self.get_logger()
        if   level == 'debug': logger.debug(msg)
        elif level == 'info':  logger.info(msg)
        elif level == 'warn':  logger.warn(msg)
        else:                  logger.error(msg)

    def cb_markers(self, msg: ArucoMarkers):
        """
        Arucoマーカー情報を受信するたびに、
        Base(ID 4) と Tip(ID 0) の最新状態を更新する。
        """
        self.last_aruco_stamp = msg.header.stamp
        ids = list(msg.marker_ids)
        poses = msg.poses

        # Base (ID 4) の検出 -> T_BC (Base <- Camera) を計算
        if self.base_id in ids:
            i = ids.index(self.base_id)
            p: Pose = poses[i]
            t = np.array([p.position.x, p.position.y, p.position.z])
            q = p.orientation
            R = rot_from_quat(q.w, q.x, q.y, q.z)
            T_CB = T_from_Rt(R, t) # Camera -> Base 座標系
            self.T_BC = T_inv(T_CB) # Base -> Camera 座標系
        
        # Tip (ID 0) の検出 -> カメラ座標系でのPoseを保持
        if self.tip_id in ids:
            j = ids.index(self.tip_id)
            self.tip_pose_C = poses[j]
        else:
            self.tip_pose_C = None # 見失ったら None にする

    def cb_goal(self, msg: PointStamped):
        """
        /hose/goal_point を受信したら、誤差を計算して記録する (メイン処理)
        """
        
        # --- 1. 必要な情報が揃っているかチェック ---
        if self.T_BC is None:
            self._log_throttle('warn', 'no_base',
                               "Base transform (ID 4) not visible. Cannot log error.", 2.0)
            return
        if self.tip_pose_C is None:
            self._log_throttle('warn', 'no_tip',
                               "Tip (ID 0) not visible. Cannot log error.", 2.0)
            return
        if self.last_aruco_stamp is None:
            self._log_throttle('warn', 'no_aruco',
                               "No Aruco data received yet. Cannot log error.", 2.0)
            return

        # --- 2. 目標座標 (Goal) を取得 ---
        # 重要：Goalの座標(msg)は、ID 4 (Base) 座標系で送られてきている前提
        goal_p_B = np.array([msg.point.x, msg.point.y, msg.point.z])
        goal_frame_id = msg.header.frame_id
        goal_stamp = msg.header.stamp

        # (念のため) Base座標系で送られているか確認
        # ★ 修正: warn_throttle -> _log_throttle
        if 'base' not in goal_frame_id.lower() and 'map' not in goal_frame_id.lower():
            log_msg = (f"Goal frame_id is '{goal_frame_id}'. "
                       f"Make sure this is the 'base' (ID 4) frame!")
            self._log_throttle('warn', 'goal_frame_warn', log_msg, 2.0)

        # --- 3. 現在の先端座標 (Actual) を計算 ---
        # Tip (ID 0) のカメラ座標系での位置
        pC_pos = self.tip_pose_C.position
        pC_vec = np.array([pC_pos.x, pC_pos.y, pC_pos.z, 1.0])
        
        # T_BC を使って Base (ID 4) 座標系に変換
        pB_vec = self.T_BC @ pC_vec
        actual_p_B = pB_vec[:3]

        # --- 4. 誤差 (ユークリッド距離) を計算 ---
        error_vec = actual_p_B - goal_p_B
        error_dist_m = np.linalg.norm(error_vec)

        # --- 5. CSVに書き込み ---
        goal_stamp_sec = goal_stamp.sec + goal_stamp.nanosec * 1e-9
        aruco_stamp_sec = self.last_aruco_stamp.sec + self.last_aruco_stamp.nanosec * 1e-9

        row = [
            f"{goal_stamp_sec:.6f}",    # Goal受信時刻
            f"{aruco_stamp_sec:.6f}",   # 最後にArucoを見た時刻
            goal_frame_id,              # GoalのフレームID (確認用)
            f"{goal_p_B[0]:.6f}", f"{goal_p_B[1]:.6f}", f"{goal_p_B[2]:.6f}", # Goal(x,y,z)
            f"{actual_p_B[0]:.6f}", f"{actual_p_B[1]:.6f}", f"{actual_p_B[2]:.6f}", # Actual(x,y,z)
            f"{error_dist_m:.6f}"       # 誤差(m)
        ]
        
        try:
            with open(self.csv_path, 'a', newline='') as f:
                w = csv.writer(f)
                w.writerow(row)
            
            self.get_logger().info(
                f"Error Logged: Goal[{goal_p_B[0]:.3f}, {goal_p_B[1]:.3f}, {goal_p_B[2]:.3f}] | "
                f"Actual[{actual_p_B[0]:.3f}, {actual_p_B[1]:.3f}, {actual_p_B[2]:.3f}] | "
                f"Error: {error_dist_m:.4f} m"
            )
        except Exception as e:
            self.get_logger().error(f"Failed to write to CSV: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = ErrorLoggerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()