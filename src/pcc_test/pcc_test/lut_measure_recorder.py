#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import csv, os
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Pose
from aruco_interfaces.msg import ArucoMarkers
from std_srvs.srv import Trigger

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

class PCCMeasureRecorder(Node):
    """
    手動操作中の実測データをCSV保存:
      - 先端Aruco(id=0)の位置を unify_frame へ変換して記録
      - /motor_current_angles (deg) と、初回キャプチャのゼロ値も保存
    """
    def __init__(self):
        super().__init__('pcc_measure_recorder')

        # ---- params ----
        self.declare_parameter('csv_path', '/home/matsunaga-h/pickup_ws/src/pcc_test/pcc_test/lut_csv/pcc_measure.csv')
        self.declare_parameter('mode', 'timer')  # 'timer' or 'trigger'
        self.declare_parameter('record_hz', 5.0) # mode=timer の周期
        self.declare_parameter('unify_frame', 'camera_color_optical_frame')  # or 'base'
        self.declare_parameter('camera_frame', 'camera_color_optical_frame')
        self.declare_parameter('base_marker_id', 4)
        self.declare_parameter('tip_marker_id', 0)
        self.declare_parameter('motor_indices', [6,7,8])  # 保存したいインデックス

        # ---- get ----
        self.csv_path     = self.get_parameter('csv_path').value
        self.mode         = self.get_parameter('mode').value.lower()
        self.hz           = float(self.get_parameter('record_hz').value)
        self.unify_frame  = self.get_parameter('unify_frame').value
        self.cam_frame    = self.get_parameter('camera_frame').value
        self.base_id      = int(self.get_parameter('base_marker_id').value)
        self.tip_id       = int(self.get_parameter('tip_marker_id').value)
        self.motor_idx    = list(self.get_parameter('motor_indices').value)

        # ---- state ----
        self.T_CB = None  # C←B
        self.T_BC = None  # B←C
        self.tip_pose_C = None  # camera系の id0 pose
        self.last_motor_deg = None
        self.zero_deg = None

        # ---- I/O ----
        self.sub_mark = self.create_subscription(ArucoMarkers, '/aruco/markers', self.cb_markers, 10)
        self.sub_mot  = self.create_subscription(Float32MultiArray, '/motor_current_angles', self.cb_motor, 10)

        if self.mode == 'timer':
            period = max(0.01, 1.0/max(1e-6, self.hz))
            self.timer = self.create_timer(period, self.on_timer)
            self.get_logger().info(f"Recording in timer mode at {self.hz} Hz")
        else:
            self.srv = self.create_service(Trigger, '/pcc_record/trigger', self.on_trigger)
            self.get_logger().info("Recording in trigger mode (call /pcc_record/trigger to snapshot)")

        self._ensure_header()

        self.get_logger().info(
            f"PCCMeasureRecorder ready. csv='{self.csv_path}', unify_frame='{self.unify_frame}', "
            f"idx={self.motor_idx}, camera_frame='{self.cam_frame}', base_id={self.base_id}, tip_id={self.tip_id}"
        )
        self._last_log_times = {}  # throttle用


    # ---------- subscribers ----------
    def cb_markers(self, msg: ArucoMarkers):
        if len(msg.marker_ids) == 0: return
        ids = list(msg.marker_ids)
        # base
        if self.base_id in ids:
            i = ids.index(self.base_id)
            p: Pose = msg.poses[i]
            t = np.array([p.position.x, p.position.y, p.position.z], float)
            q = p.orientation
            R = rot_from_quat(q.w,q.x,q.y,q.z)
            self.T_CB = T_from_Rt(R,t)
            self.T_BC = T_inv(self.T_CB)
        # tip (camera系のまま保持)
        if self.tip_id in ids:
            j = ids.index(self.tip_id)
            self.tip_pose_C = msg.poses[j]
        else:
            self.tip_pose_C = None

    def cb_motor(self, msg: Float32MultiArray):
        arr = np.array(msg.data, float)
        self.last_motor_deg = arr
        if self.zero_deg is None:
            self.zero_deg = arr.copy()
            self.get_logger().info("Captured zero (deg) from /motor_current_angles.")

    # ---------- timer / trigger ----------
    def on_timer(self):
        self._maybe_write_row()

    def on_trigger(self, req, resp):
        ok = self._maybe_write_row()
        resp.success = bool(ok)
        resp.message = "recorded" if ok else "not recorded (tip or transform missing / no motor)"
        return resp

    # ---------- core ----------
    def _maybe_write_row(self):
        # 必要データが揃っているか
        if self.last_motor_deg is None or self.zero_deg is None:
            self.get_logger().warn_throttle(2.0, "no motor/zero yet; skip")
            return False
        if self.tip_pose_C is None:
            self.get_logger().warn_throttle(2.0, "tip(id=0) not visible; skip")
            return False

        # tip を unify_frame に変換
        pC = np.array([self.tip_pose_C.position.x,
                       self.tip_pose_C.position.y,
                       self.tip_pose_C.position.z, 1.0], float)
        if self.unify_frame == self.cam_frame:
            pU = pC
        elif self.unify_frame.lower() == 'base':
            if self.T_BC is None:
                self.get_logger().warn_throttle(2.0, "no base transform; skip")
                return False
            pU = self.T_BC @ pC
        else:
            # それ以外は未対応（必要なら拡張）
            self._log_throttle('warn', 'tip_not_visible', "tip(id=0) not visible; skip", 2.0)
            return False

        # 取り出すモータ角（指定 index）
        try:
            m = [float(self.last_motor_deg[i]) for i in self.motor_idx]
            z = [float(self.zero_deg[i])       for i in self.motor_idx]
        except Exception:
            self.get_logger().error("motor_indices out of range")
            return False

        # 書き込み
        st = self.get_clock().now().nanoseconds * 1e-9
        row = [f"{st:.6f}", self.unify_frame,
               f"{pU[0]:.6f}", f"{pU[1]:.6f}", f"{pU[2]:.6f}"] + \
              [f"{v:.4f}" for v in m] + [f"{v:.4f}" for v in z]

        with open(self.csv_path, 'a', newline='') as f:
            w = csv.writer(f)
            w.writerow(row)

        self.get_logger().info(
            f"save: frame={self.unify_frame} tip=({float(pU[0]):.3f},{float(pU[1]):.3f},{float(pU[2]):.3f}) "
            f"m={list(map(lambda x: round(x,2), m))} z={list(map(lambda x: round(x,2), z))}"
        )
        return True

    def _ensure_header(self):
        if os.path.exists(self.csv_path) and os.path.getsize(self.csv_path) > 0:
            return
        with open(self.csv_path, 'w', newline='') as f:
            w = csv.writer(f)
            cols = ['stamp','frame','x','y','z']
            cols += [f"m{idx}" for idx in self.motor_idx]
            cols += [f"z{idx}" for idx in self.motor_idx]
            w.writerow(cols)
        self.get_logger().info(f"CSV header written to {self.csv_path}")

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


def main():
    rclpy.init()
    rclpy.spin(PCCMeasureRecorder())
    rclpy.shutdown()

if __name__ == '__main__':
    main()
