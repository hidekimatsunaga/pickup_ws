#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import math
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import PointStamped, Pose
from aruco_interfaces.msg import ArucoMarkers  # ids:int32[], poses:Pose[]
from std_srvs.srv import Trigger  # <-- reset用

# ---- PCC centerline on 2D plane (x-y) ----
def pcc_centerline_xy(theta, L, tip_extra=0.0, n_per_seg=80, psi0=0.0):
    xs = [0.0]; ys = [0.0]
    psi = psi0
    x = 0.0; y = 0.0
    for th, Li in zip(theta, L):
        k = th / Li if abs(Li) > 1e-12 else 0.0
        if abs(k) < 1e-10:
            x += math.cos(psi)*Li
            y += math.sin(psi)*Li
            xs.append(x); ys.append(y)
        else:
            R = 1.0/k
            psi_new = psi + th
            x += R*(math.sin(psi_new)-math.sin(psi))
            y += -R*(math.cos(psi_new)-math.cos(psi))
            psi = psi_new
            xs.append(x); ys.append(y)
    if tip_extra > 0.0:
        x += math.cos(psi)*tip_extra
        y += math.sin(psi)*tip_extra
        xs.append(x); ys.append(y)
    return np.array(xs), np.array(ys)

def theta_from_motor_deg_constrained(motor_deg, zero_deg, idxs, r_wire, r_pulley):
    dalpha_deg = np.array([motor_deg[idxs[0]]-zero_deg[idxs[0]],
                           motor_deg[idxs[1]]-zero_deg[idxs[1]],
                           motor_deg[idxs[2]]-zero_deg[idxs[2]]], float)
    dls = np.radians(dalpha_deg) * r_pulley
    t1 = dls[0]/r_wire
    t2 = (dls[1]-dls[0])/r_wire
    t3 = (dls[2]-dls[1])/r_wire
    return np.array([t1,t2,t3], float)

def rot_from_quat(w, x, y, z):
    return np.array([
        [1-2*(y*y+z*z), 2*(x*y - z*w), 2*(x*z + y*w)],
        [2*(x*y + z*w), 1-2*(x*x+z*z), 2*(y*z - x*w)],
        [2*(x*z - y*w), 2*(y*z + x*w), 1-2*(x*x+y*y)]
    ], float)

def T_from_Rt(R, t):
    T = np.eye(4); T[:3,:3]=R; T[:3,3]=t; return T
def T_inv(T):
    R=T[:3,:3]; t=T[:3,3]
    Ti=np.eye(4); Ti[:3,:3]=R.T; Ti[:3,3]=-R.T@t; return Ti

class PCCTipPlanePublisher(Node):
    """
    PCC仮定の先端位置（モデル）を B系マーカー面(B.x–B.y)上の (u,v,0) としてpublish。
    併せて実測先端(id0)も B系へ変換し (u_meas,v_meas,0) をpublish。
    """
    def __init__(self):
        super().__init__('pcc_tip_plane_publisher')

        # パラメータ
        self.declare_parameter('L', [0.20, 0.20, 0.20])
        self.declare_parameter('tip_extra', 0.10)
        self.declare_parameter('wire_radius', 0.019)
        self.declare_parameter('pulley_radius', 0.008)
        self.declare_parameter('motor_indices', [6,7,8])
        self.declare_parameter('base_marker_id', 4)
        self.declare_parameter('tip_marker_id', 0)
        self.declare_parameter('camera_frame', 'camera_color_optical_frame')
        self.declare_parameter('publish_hz', 20.0)
        self.declare_parameter('psi0', np.pi)  # -x で始めるなら π を指定
        self.declare_parameter('mirror_x_only', False)

        # 追加：ベース固定と実測ジャンプ抑制
        self.declare_parameter('lock_base_on_first', True)     # 初回検出で固定
        self.declare_parameter('meas_jump_thresh_m', 0.08)     # 80 mm

        self.L = np.array(self.get_parameter('L').value, float)
        self.tip_extra = float(self.get_parameter('tip_extra').value)
        self.r_wire = float(self.get_parameter('wire_radius').value)
        self.r_pulley = float(self.get_parameter('pulley_radius').value)
        self.motor_idx = list(self.get_parameter('motor_indices').value)
        self.base_id = int(self.get_parameter('base_marker_id').value)
        self.tip_id = int(self.get_parameter('tip_marker_id').value)
        self.cam_frame = self.get_parameter('camera_frame').value
        self.psi0 = float(self.get_parameter('psi0').value)
        self.mirror_x_only = bool(self.get_parameter('mirror_x_only').value)

        self.lock_base_on_first = bool(self.get_parameter('lock_base_on_first').value)
        self.meas_jump_thresh = float(self.get_parameter('meas_jump_thresh_m').value)

        # 状態
        self.zero_deg = None
        self.last_motor_deg = None
        self.T_CB = None  # C←B
        self.T_BC = None  # B←C
        self.tip_pose_C = None
        self.base_locked = False
        self.prev_meas_uv = None

        # resetサービス
        self.srv_reset = self.create_service(Trigger, 'reset_base_lock', self.on_reset_base)

        # I/O
        self.sub_mot = self.create_subscription(Float32MultiArray, '/motor_current_angles', self.cb_motor, 10)
        self.sub_mark = self.create_subscription(ArucoMarkers, '/aruco/markers', self.cb_markers, 10)
        self.pub_model_uv = self.create_publisher(PointStamped, '/pcc_tip_on_base_plane', 10)
        self.pub_meas_uv  = self.create_publisher(PointStamped, '/meas_tip_on_base_plane', 10)

        period = 1.0 / max(1e-6, float(self.get_parameter('publish_hz').value))
        self.timer = self.create_timer(period, self.on_timer)
        self.get_logger().info('PCCTipPlanePublisher ready.')

    # --- callbacks ---
    def on_reset_base(self):
        self.base_locked = False
        self.T_CB = None
        self.T_BC = None
        self.prev_meas_uv = None

    def on_reset_base(self, req, res):
        self.on_reset_base()
        res.success = True
        res.message = 'Base lock reset. Will capture on next valid detection.'
        self.get_logger().warn(res.message)
        return res

    def cb_motor(self, msg: Float32MultiArray):
        arr = np.array(msg.data, float)
        self.last_motor_deg = arr
        if self.zero_deg is None:
            self.zero_deg = arr.copy()
            self.get_logger().info('Captured zero (deg) from /motor_current_angles.')

    def cb_markers(self, msg: ArucoMarkers):
        if len(msg.marker_ids) == 0:
            return
        try:
            ids = list(msg.marker_ids)

            # --- base(id4) ---
            if (not self.base_locked) and (self.base_id in ids):
                i = ids.index(self.base_id)
                p: Pose = msg.poses[i]
                t = np.array([p.position.x, p.position.y, p.position.z], float)
                q = p.orientation
                R = rot_from_quat(q.w, q.x, q.y, q.z)
                self.T_CB = T_from_Rt(R, t)  # C←B
                self.T_BC = T_inv(self.T_CB)
                if self.lock_base_on_first:
                    self.base_locked = True
                    self.get_logger().info('Base transform captured and locked.')

            # --- tip(id0) camera系で保持 ---
            self.tip_pose_C = None
            if self.tip_id in ids:
                j = ids.index(self.tip_id)
                self.tip_pose_C = msg.poses[j]

        except Exception as e:
            self.get_logger().warn(f'/aruco/markers parse error: {e}')

    # --- main loop ---
    def on_timer(self):
        if self.last_motor_deg is None or self.zero_deg is None:
            return

        # モデル先端 (u,v) in B-plane
        theta = theta_from_motor_deg_constrained(self.last_motor_deg, self.zero_deg,
                                                 self.motor_idx, self.r_wire, self.r_pulley)
        X, Y = pcc_centerline_xy(theta, self.L, tip_extra=self.tip_extra, psi0=self.psi0)
        u_tip = float(X[-1]); v_tip = float(Y[-1])
        if self.mirror_x_only:
            u_tip = -u_tip  # ← 反転をpublish前に反映（バグ修正）

        pm = PointStamped()
        pm.header.frame_id = 'base'
        pm.header.stamp = self.get_clock().now().to_msg()
        pm.point.x = u_tip
        pm.point.y = v_tip
        pm.point.z = 0.0
        self.pub_model_uv.publish(pm)

        # 実測 tip を B系に変換し平面に投影（比較用）
        if self.T_BC is not None and self.tip_pose_C is not None:
            pC = np.array([self.tip_pose_C.position.x,
                           self.tip_pose_C.position.y,
                           self.tip_pose_C.position.z, 1.0], float)
            pB = self.T_BC @ pC
            uv = np.array([float(pB[0]), float(pB[1])], float)

            # ジャンプ閾値でスパイク抑制
            ok = True
            if self.prev_meas_uv is not None:
                if np.linalg.norm(uv - self.prev_meas_uv) > self.meas_jump_thresh:
                    ok = False
                    self.get_logger().warn(
                        f'Meas jump rejected: {np.linalg.norm(uv - self.prev_meas_uv):.3f} m > {self.meas_jump_thresh:.3f} m'
                    )

            if ok:
                pe = PointStamped()
                pe.header.frame_id = 'base'
                pe.header.stamp = pm.header.stamp
                pe.point.x = uv[0]
                pe.point.y = uv[1]
                pe.point.z = 0.0
                self.pub_meas_uv.publish(pe)
                self.prev_meas_uv = uv

def main():
    rclpy.init()
    node = PCCTipPlanePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
