#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import math, csv, os, bisect
import numpy as np
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PointStamped, Pose
from aruco_interfaces.msg import ArucoMarkers  # ids:int32[], poses:Pose[]

# ===== ユーティリティ =====
def rot_from_quat(w, x, y, z):
    """クォータニオン -> 回転行列（正規化込み）"""
    n = math.sqrt(w*w + x*x + y*y + z*z)
    if n == 0.0:
        return np.eye(3)
    w, x, y, z = w/n, x/n, y/n, z/n
    return np.array([
        [1-2*(y*y+z*z), 2*(x*y - z*w), 2*(x*z + y*w)],
        [2*(x*y + z*w), 1-2*(x*x+z*z), 2*(y*z - x*w)],
        [2*(x*z - y*w), 2*(y*z + x*w), 1-2*(x*x+y*y)]
    ], float)

def T_from_Rt(R, t):
    T = np.eye(4); T[:3, :3] = R; T[:3, 3] = t; return T

def T_inv(T):
    R = T[:3, :3]; t = T[:3, 3]
    Ti = np.eye(4); Ti[:3, :3] = R.T; Ti[:3, 3] = -R.T @ t
    return Ti

def isfinite_vec(v):
    return np.all(np.isfinite(v))

# ===== ノード =====
class PCCErrorLogger(Node):
    def __init__(self):
        super().__init__('pcc_error_logger')

        # --- パラメータ ---
        self.declare_parameter('base_marker_id', 4)                 # 基準マーカ (id4)
        self.declare_parameter('tip_marker_id', 0)                  # 先端マーカ (id0)
        self.declare_parameter('csv_filename', 'pcc_error_log.csv') # 出力CSV
        self.declare_parameter('camera_frame_hint', 'camera')       # camera系判定キーワード
        self.declare_parameter('buffer_maxlen', 1000)               # ArUcoバッファ長

        # 取得
        self.base_id = int(self.get_parameter('base_marker_id').value)
        self.tip_id = int(self.get_parameter('tip_marker_id').value)
        self.csv_filename = self.get_parameter('csv_filename').value
        self.cam_key = (self.get_parameter('camera_frame_hint').value or 'camera').lower()
        self.buf_maxlen = int(self.get_parameter('buffer_maxlen').value)

        # 状態
        self.T_BC = None          # B←C (ベース←カメラ)
        self.ts_list = []         # ArUco時刻の昇順リスト [float sec]
        self.pB_list = []         # 各時刻の tip(base) 位置 [[x,y,z], ...]
        self.tip_detected_once = False
        self.base_detected_once = False

        # CSVファイル準備（開きっぱなし）
        try:
            filepath = os.path.join(os.getcwd(), self.csv_filename)
            self.csv_file = open(filepath, 'w', newline='')
            self.csv_writer = csv.writer(self.csv_file)
            self.csv_writer.writerow([
                'timestamp_sec',
                'target_x', 'target_y', 'target_z',
                'actual_x', 'actual_y', 'actual_z',
                'error_dist', 'error_x', 'error_y', 'error_z'
            ])
            self.get_logger().info(f"Logging errors to: {filepath}")
        except Exception as e:
            self.get_logger().error(f"Failed to open CSV file: {e}")
            raise

        # I/O
        self.sub_mark = self.create_subscription(
            ArucoMarkers, '/aruco/markers', self.cb_markers, 10)
        self.sub_target = self.create_subscription(
            PointStamped, '/pcc_target', self.cb_target_and_log, 10)

    def destroy_node(self):
        # ノード破棄時にCSVを閉じる
        try:
            if hasattr(self, 'csv_file') and self.csv_file:
                self.csv_file.close()
                self.get_logger().info("CSV file closed.")
        finally:
            super().destroy_node()

    # ---- ArUco ----
    def cb_markers(self, msg: ArucoMarkers):
        if len(msg.marker_ids) == 0:
            return

        ids = list(msg.marker_ids)
        poses_C = msg.poses  # カメラ座標系でのマーカ姿勢

        try:
            # 1) Base(id4) → T_BC 更新（Camera←Base の T_C_B を反転）
            if self.base_id in ids:
                i = ids.index(self.base_id)
                base_pose: Pose = poses_C[i]
                t = np.array([base_pose.position.x, base_pose.position.y, base_pose.position.z], float)
                q = base_pose.orientation
                R = rot_from_quat(q.w, q.x, q.y, q.z)
                if not (isfinite_vec(t) and isfinite_vec(R)):
                    self.get_logger().warning("Non-finite base pose ignored.")
                    return
                T_CB = T_from_Rt(R, t)   # C←B
                self.T_BC = T_inv(T_CB)  # B←C
                if not self.base_detected_once:
                    self.get_logger().info(f"Base (id={self.base_id}) detected. T_BC ready.")
                    self.base_detected_once = True

            # 2) Tip(id0) を B系に変換してバッファへ
            if self.tip_id in ids and self.T_BC is not None:
                j = ids.index(self.tip_id)
                tip_pose: Pose = poses_C[j]
                pC = np.array([tip_pose.position.x, tip_pose.position.y, tip_pose.position.z, 1.0], float)
                if not isfinite_vec(pC):
                    self.get_logger().warning("Non-finite tip pose ignored.")
                    return

                pB_h = self.T_BC @ pC
                pB = pB_h[:3]
                if not isfinite_vec(pB):
                    self.get_logger().warning("Non-finite tip(base) ignored.")
                    return

                ts = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
                k = bisect.bisect_right(self.ts_list, ts)
                self.ts_list.insert(k, ts)
                self.pB_list.insert(k, pB)

                # バッファ長管理
                if len(self.ts_list) > self.buf_maxlen:
                    self.ts_list.pop(0)
                    self.pB_list.pop(0)

                if not self.tip_detected_once:
                    self.get_logger().info(f"Tip (id={self.tip_id}) detected.")
                    self.tip_detected_once = True

        except Exception as e:
            self.get_logger().warning(f'/aruco/markers parse error: {e}')

    # ---- 目標受信&ログ ----
    def cb_target_and_log(self, msg: PointStamped):
        # 1) 目標のフレームを判定して base系へ
        frame = (msg.header.frame_id or '').lower()
        ts_goal = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

        if 'base' in frame:
            p_target = np.array([msg.point.x, msg.point.y, msg.point.z], float)
        elif self.cam_key in frame or 'camera' in frame:
            if self.T_BC is None:
                self.get_logger().warning("Target in camera frame, but T_BC not ready. Skipped.")
                return
            pC = np.array([msg.point.x, msg.point.y, msg.point.z, 1.0], float)
            if not isfinite_vec(pC):
                self.get_logger().warning("Non-finite target(camera) skipped.")
                return
            p_target = (self.T_BC @ pC)[:3]
        elif 'map' in frame:
            self.get_logger().warning("Target in 'map' frame not supported here (no tf2). Skipped.")
            return
        else:
            # 厳密運用なら return してもよい
            self.get_logger().warning(f"Unknown frame_id '{msg.header.frame_id}'. Assuming base.")
            p_target = np.array([msg.point.x, msg.point.y, msg.point.z], float)

        if not isfinite_vec(p_target):
            self.get_logger().warning("Non-finite target(base) skipped.")
            return

        # 2) tip(base) の最新バッファを時刻整合（最近傍＋線形補間）
        if len(self.ts_list) == 0:
            self.get_logger().warning("No tip samples buffered. Skipped.")
            return

        idx = bisect.bisect_left(self.ts_list, ts_goal)
        if idx == 0:
            used_ts = self.ts_list[0]
            p_actual = self.pB_list[0]
        elif idx >= len(self.ts_list):
            used_ts = self.ts_list[-1]
            p_actual = self.pB_list[-1]
        else:
            t0, t1 = self.ts_list[idx-1], self.ts_list[idx]
            p0, p1 = self.pB_list[idx-1], self.pB_list[idx]
            denom = (t1 - t0) if (t1 - t0) != 0.0 else 1e-9
            w = (ts_goal - t0) / denom
            p_actual = (1.0 - w) * p0 + w * p1
            used_ts = ts_goal

        if not isfinite_vec(p_actual):
            self.get_logger().warning("Non-finite actual(base) skipped.")
            return

        # 3) 誤差計算（target - actual）
        err_vec = p_target - p_actual
        err_dist = float(np.linalg.norm(err_vec))

        # 4) CSVに追記
        try:
            self.csv_writer.writerow([
                f"{ts_goal:.6f}",
                f"{p_target[0]:.6f}", f"{p_target[1]:.6f}", f"{p_target[2]:.6f}",
                f"{p_actual[0]:.6f}", f"{p_actual[1]:.6f}", f"{p_actual[2]:.6f}",
                f"{err_dist:.6f}",
                f"{err_vec[0]:.6f}", f"{err_vec[1]:.6f}", f"{err_vec[2]:.6f}"
            ])
            # 必要なら flush
            # self.csv_file.flush()

            # 表示はx–zだけで十分なら y を省略してOK
            self.get_logger().info(
                f"Logged: Target=({p_target[0]:.3f},{p_target[2]:.3f}) | "
                f"Actual=({p_actual[0]:.3f},{p_actual[2]:.3f}) | "
                f"Err={err_dist:.4f}"
            )
        except Exception as e:
            self.get_logger().error(f"Failed to write to CSV: {e}")

# ===== エントリポイント =====
def main():
    rclpy.init()
    node = PCCErrorLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
