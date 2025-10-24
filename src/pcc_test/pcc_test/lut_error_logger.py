#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import csv, os, math, bisect
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped, Pose
from aruco_interfaces.msg import ArucoMarkers

# ================= ユーティリティ =================
def rot_from_quat(w, x, y, z):
    """クォータニオン -> 3x3回転行列（正規化込み）"""
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

# ================= ノード =================
class ErrorLoggerNode(Node):
    """
    /hose/goal_point を受信するたびに、
    その時点（に最も近い時刻）の Aruco(ID 0) の base(ID 4) 基準座標を取得し、
    目標値との誤差をCSVに記録する。
    - /aruco/markers は「カメラ座標系で観測したマーカ姿勢」を与える前提。
      つまり、Camera <- Base の T_C_B = [R|t] が得られる。
      よって Base <- Camera の T_B_C = (T_C_B)^(-1) を使って
      カメラ座標の点 p_C を base座標 p_B へ変換する。
    """
    def __init__(self):
        super().__init__('pcc_error_logger')

        # ---- Parameters ----
        self.declare_parameter('csv_path', '~/pickup_ws/src/pcc_test/error_log.csv')
        self.declare_parameter('base_marker_id', 4)
        self.declare_parameter('tip_marker_id', 0)
        self.declare_parameter('camera_frame_hint', 'camera')  # goalがcamera系かを判定するためのキーワード
        self.declare_parameter('buffer_maxlen', 1000)          # arucoバッファ長

        # ---- Get Parameters ----
        self.csv_path = os.path.expanduser(self.get_parameter('csv_path').value)
        self.base_id = int(self.get_parameter('base_marker_id').value)
        self.tip_id = int(self.get_parameter('tip_marker_id').value)
        self.cam_key = (self.get_parameter('camera_frame_hint').value or 'camera').lower()
        self.buf_maxlen = int(self.get_parameter('buffer_maxlen').value)

        # ---- State ----
        self.T_BC = None                # Base <- Camera の変換（最新）
        self.last_aruco_stamp = None    # 直近の /aruco/markers の時刻

        # 時系列バッファ（時刻リストと対応の位置ベクトルリスト）
        # ts_list: 昇順のfloat秒, pB_list: shape=(N,3)
        self.ts_list = []               # [t0, t1, ...] (float sec)
        self.pB_list = []               # [[x,y,z], ...]
        self._last_log_times = {}       # スロットリング用

        # ---- I/O ----
        self.sub_markers = self.create_subscription(
            ArucoMarkers, '/aruco/markers', self.cb_markers, 10)
        self.sub_goal = self.create_subscription(
            PointStamped, '/hose/goal_point', self.cb_goal, 10)

        self._ensure_header()
        self.get_logger().info(f"ErrorLoggerNode ready. Logging to: {self.csv_path}")

    # ---------- ログスロットリング ----------
    def _log_throttle(self, level: str, key: str, msg: str, period_sec: float = 2.0):
        now = self.get_clock().now().nanoseconds * 1e-9
        last = self._last_log_times.get(key, 0.0)
        if now - last < period_sec:
            return
        self._last_log_times[key] = now
        logger = self.get_logger()
        if   level == 'debug': logger.debug(msg)
        elif level == 'info':  logger.info(msg)
        elif level == 'warn':  logger.warning(msg)
        else:                  logger.error(msg)

    # ---------- CSV ヘッダ ----------
    def _ensure_header(self):
        if os.path.exists(self.csv_path) and os.path.getsize(self.csv_path) > 0:
            return
        try:
            with open(self.csv_path, 'w', newline='') as f:
                w = csv.writer(f)
                w.writerow([
                    'goal_stamp',
                    'aruco_used_stamp',
                    'goal_frame_id',
                    'goal_x_B', 'goal_y_B', 'goal_z_B',
                    'actual_x_B', 'actual_y_B', 'actual_z_B',
                    'error_x', 'error_y', 'error_z',
                    'error_norm_m'
                ])
            self.get_logger().info(f"CSV header written to {self.csv_path}")
        except Exception as e:
            self.get_logger().error(f"Failed to write CSV header: {e}")

    # ---------- ArUcoコールバック ----------
    def cb_markers(self, msg: ArucoMarkers):
        self.last_aruco_stamp = msg.header.stamp
        ids = list(msg.marker_ids)
        poses = msg.poses  # 各Poseはカメラ座標系でのマーカ姿勢

        # 1) Base(ID4) から T_BC を更新（Camera <- Base の T_C_B を反転）
        if self.base_id in ids:
            i = ids.index(self.base_id)
            pB_pose: Pose = poses[i]
            t = np.array([pB_pose.position.x, pB_pose.position.y, pB_pose.position.z], float)
            q = pB_pose.orientation
            R = rot_from_quat(q.w, q.x, q.y, q.z)
            if not (isfinite_vec(t) and isfinite_vec(R)):
                self._log_throttle('warn', 'bad_base', "Non-finite base pose ignored.", 2.0)
                return
            T_CB = T_from_Rt(R, t)   # Camera <- Base
            self.T_BC = T_inv(T_CB)  # Base   <- Camera

        # 2) Tip(ID0) が見えていて、かつ T_BC があれば tipをbase系に変換してバッファへ
        if self.tip_id in ids and self.T_BC is not None:
            j = ids.index(self.tip_id)
            tip_pose: Pose = poses[j]
            pC = np.array([tip_pose.position.x, tip_pose.position.y, tip_pose.position.z, 1.0], float)
            if not isfinite_vec(pC):
                self._log_throttle('warn', 'bad_tip', "Non-finite tip pose ignored.", 2.0)
                return

            pB_h = self.T_BC @ pC
            pB = pB_h[:3]
            if not isfinite_vec(pB):
                self._log_throttle('warn', 'bad_tipB', "Non-finite tip(base) ignored.", 2.0)
                return

            ts = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

            # 昇順挿入（bisect）
            k = bisect.bisect_right(self.ts_list, ts)
            self.ts_list.insert(k, ts)
            self.pB_list.insert(k, pB)

            # バッファ長管理
            if len(self.ts_list) > self.buf_maxlen:
                self.ts_list.pop(0)
                self.pB_list.pop(0)

    # ---------- 目標受信コールバック ----------
    def cb_goal(self, msg: PointStamped):
        # --- 前提チェック ---
        if self.T_BC is None:
            self._log_throttle('warn', 'no_base', "Base transform (ID4) not ready.", 2.0)
            return
        if len(self.ts_list) == 0:
            self._log_throttle('warn', 'no_tip', "No tip samples buffered.", 2.0)
            return

        # --- 目標の座標系判定と base系への変換 ---
        frame = (msg.header.frame_id or '').lower()
        goal_stamp = msg.header.stamp
        goal_ts = goal_stamp.sec + goal_stamp.nanosec * 1e-9

        if 'base' in frame:
            goal_p_B = np.array([msg.point.x, msg.point.y, msg.point.z], float)
        elif self.cam_key in frame or 'camera' in frame:
            pC = np.array([msg.point.x, msg.point.y, msg.point.z, 1.0], float)
            if not isfinite_vec(pC):
                self._log_throttle('warn', 'bad_goal', "Non-finite goal(camera) skipped.", 2.0)
                return
            goal_p_B = (self.T_BC @ pC)[:3]
        elif 'map' in frame:
            # tf2 未導入のため map→base 変換は未対応。必要なら tf2_ros を導入してください。
            self._log_throttle('warn', 'map_goal',
                               "Goal in 'map' frame not supported here (no tf2). Skipped.", 2.0)
            return
        else:
            self._log_throttle('warn', 'unknown_frame',
                               f"Unknown goal frame_id '{msg.header.frame_id}'. Expected base/camera.", 2.0)
            return

        if not isfinite_vec(goal_p_B):
            self._log_throttle('warn', 'bad_goalB', "Non-finite goal(base) skipped.", 2.0)
            return

        # --- 目標時刻に最も近い tip(base) を取得（線形補間） ---
        idx = bisect.bisect_left(self.ts_list, goal_ts)
        if idx == 0:
            used_ts = self.ts_list[0]
            actual_p_B = self.pB_list[0]
        elif idx >= len(self.ts_list):
            used_ts = self.ts_list[-1]
            actual_p_B = self.pB_list[-1]
        else:
            t0, t1 = self.ts_list[idx-1], self.ts_list[idx]
            p0, p1 = self.pB_list[idx-1], self.pB_list[idx]
            # 線形補間
            denom = (t1 - t0) if (t1 - t0) != 0.0 else 1e-9
            w = (goal_ts - t0) / denom
            actual_p_B = (1.0 - w) * p0 + w * p1
            used_ts = goal_ts  # 補間なので、実質的に目標時刻で評価

        if not isfinite_vec(actual_p_B):
            self._log_throttle('warn', 'bad_actual', "Non-finite interpolated actual skipped.", 2.0)
            return

        # --- 誤差計算（target - actual で統一） ---
        error_vec = goal_p_B - actual_p_B
        error_dist_m = float(np.linalg.norm(error_vec))

        # --- CSV 出力 ---
        try:
            with open(self.csv_path, 'a', newline='') as f:
                w = csv.writer(f)
                w.writerow([
                    f"{goal_ts:.6f}",
                    f"{used_ts:.6f}",
                    msg.header.frame_id,
                    f"{goal_p_B[0]:.6f}", f"{goal_p_B[1]:.6f}", f"{goal_p_B[2]:.6f}",
                    f"{actual_p_B[0]:.6f}", f"{actual_p_B[1]:.6f}", f"{actual_p_B[2]:.6f}",
                    f"{error_vec[0]:.6f}", f"{error_vec[1]:.6f}", f"{error_vec[2]:.6f}",
                    f"{error_dist_m:.6f}"
                ])
            self.get_logger().info(
                f"Logged: Goal[{goal_p_B[0]:.3f},{goal_p_B[1]:.3f},{goal_p_B[2]:.3f}] | "
                f"Actual[{actual_p_B[0]:.3f},{actual_p_B[1]:.3f},{actual_p_B[2]:.3f}] | "
                f"Err({error_vec[0]:.3f},{error_vec[1]:.3f},{error_vec[2]:.3f}) | "
                f"||e||={error_dist_m:.4f} m"
            )
        except Exception as e:
            self.get_logger().error(f"Failed to write to CSV: {e}")

# ================= エントリポイント =================
def main(args=None):
    rclpy.init(args=args)
    node = ErrorLoggerNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
