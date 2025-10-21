#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import math
import numpy as np
import rclpy
from rclpy.node import Node

# msgs
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import PointStamped, Pose
from aruco_interfaces.msg import ArucoMarkers  # ids:int32[], poses:Pose[]

# ===== ユーティリティ =====
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

# ===== 平面PCC（x–z平面、ψは+x基準、初期ψ=+z=π/2）=====
def pcc_forward_end(theta, L, tip_extra=0.0):
    """
    theta=[θ1,θ2,θ3], L=[L1,L2,L3] -> p_BE=[x,0,z], psi_end
    tip_extra: 最後の接線方向に付く「直線の余長」（先端まっすぐ部）
    """
    x = 0.0; z = 0.0; psi = math.pi/2.0
    for th, Li in zip(theta, L):
        k = 0.0 if abs(Li) < 1e-12 else th / Li
        if abs(k) < 1e-10:
            x += math.cos(psi) * Li
            z += math.sin(psi) * Li
        else:
            R = 1.0 / k
            psi_new = psi + th
            x += R * (math.sin(psi_new) - math.sin(psi))
            z += -R * (math.cos(psi_new) - math.cos(psi))
            psi = psi_new
    # 先端の直線余長（最後の接線方向に足す）
    if tip_extra > 0.0:
        x += math.cos(psi) * tip_extra
        z += math.sin(psi) * tip_extra
    return np.array([x, 0.0, z], float), psi

def jac_xz_wrt_theta(theta, L, tip_extra=0.0, eps=1e-6):
    p0,_ = pcc_forward_end(theta, L, tip_extra)
    f0 = np.array([p0[0], p0[2]])
    J = np.zeros((2,3))
    for i in range(3):
        th = theta.copy(); th[i] += eps
        p1,_ = pcc_forward_end(th, L, tip_extra)
        f1 = np.array([p1[0], p1[2]])
        J[:, i] = (f1 - f0)/eps
    return J

def lm_ik_2d(p_des_xz, L, tip_extra=0.0, theta0=None, iters=150, lam=1e-2):
    theta = np.zeros(3) if theta0 is None else theta0.copy()
    for _ in range(iters):
        p,_ = pcc_forward_end(theta, L, tip_extra)
        e = np.array([p_des_xz[0]-p[0], p_des_xz[1]-p[2]])
        if np.linalg.norm(e) < 1e-4:
            return theta, True
        J = jac_xz_wrt_theta(theta, L, tip_extra)
        H = J.T @ J + lam*np.eye(3); g = J.T @ e
        try:
            dtheta = np.linalg.solve(H, g)
        except np.linalg.LinAlgError:
            return theta, False
        theta += dtheta
    return theta, False

# ===== ノード =====
class PCCMoveNode(Node):
    def __init__(self):
        super().__init__('pcc_move_node')

        # --- パラメータ（あなたの実機に合わせて既定値をセット）---
        self.declare_parameter('L', [0.20, 0.20, 0.20])     # 200mm,200mm,200mm
        self.declare_parameter('tip_extra', 0.10)           # 先端直線 100mm
        self.declare_parameter('wire_radius', 0.019)        # 19mm（yマイナス側固定）
        self.declare_parameter('pulley_radius', 0.008)      # 8mm
        self.declare_parameter('motor_indices', [6,7,8])    # /motor_current_angles の index
        self.declare_parameter('cmd_topic', '/motor_angles')# ★ 出力先
        self.declare_parameter('base_marker_id', 4)         # 根本
        self.declare_parameter('camera_frame', 'camera_color_optical_frame')
        self.declare_parameter('command_units', 'deg')      # 出力単位：'deg' 固定（受信もdeg）

        # 取得
        self.L = np.array(self.get_parameter('L').value, dtype=float)
        self.tip_extra = float(self.get_parameter('tip_extra').value)
        self.r_wire = float(self.get_parameter('wire_radius').value)
        self.r_pulley = float(self.get_parameter('pulley_radius').value)
        self.motor_idx = list(self.get_parameter('motor_indices').value)
        self.cmd_topic = self.get_parameter('cmd_topic').value
        self.base_id = int(self.get_parameter('base_marker_id').value)
        self.cam_frame = self.get_parameter('camera_frame').value
        self.cmd_units = self.get_parameter('command_units').value.lower()

        # 状態
        self.zero_motor_deg = None        # 初回受信をゼロとする（度）
        self.last_angles_deg = None
        self.T_BC = None                  # B←C（ベースarucoから更新）

        # I/O
        self.sub_mot = self.create_subscription(
            Float32MultiArray, '/motor_current_angles', self.cb_motor, 10)
        self.sub_mark = self.create_subscription(
            ArucoMarkers, '/aruco/markers', self.cb_markers, 10)
        self.sub_target = self.create_subscription(
            PointStamped, '/pcc_target', self.cb_target, 10)
        self.pub_cmd = self.create_publisher(Float32MultiArray, self.cmd_topic, 10)

        self.get_logger().info(f'PCCMoveNode up. camera_frame={self.cam_frame}  cmd_topic={self.cmd_topic}')

    # --- callbacks ---
    def cb_motor(self, msg: Float32MultiArray):
        # 受信は度（deg）
        arr_deg = np.array(msg.data, dtype=float)
        self.last_angles_deg = arr_deg
        if self.zero_motor_deg is None:
            self.zero_motor_deg = arr_deg.copy()
            self.get_logger().info(f'Captured zero (deg) from /motor_current_angles (len={len(arr_deg)}).')

    def cb_markers(self, msg: ArucoMarkers):
        # ArucoMarkers: ids:int32[], poses:Pose[]
        if len(msg.marker_ids) == 0:
            return
        try:
            ids = list(msg.marker_ids)
            if self.base_id in ids:
                i = ids.index(self.base_id)
                base_pose: Pose = msg.poses[i]
                t = np.array([base_pose.position.x, base_pose.position.y, base_pose.position.z], float)
                q = base_pose.orientation
                R = rot_from_quat(q.w, q.x, q.y, q.z)
                T_CB = T_from_Rt(R, t)   # C←B
                self.T_BC = T_inv(T_CB)  # B←C
        except Exception as e:
            self.get_logger().warn(f'/aruco/markers parse error: {e}')

    def cb_target(self, msg: PointStamped):
        if self.zero_motor_deg is None:
            self.get_logger().warn('Zero not captured yet. Ignore target.')
            return

        # ---- 目標をB系に ----
        frame = (msg.header.frame_id or '').lower()
        if frame.startswith('base'):
            pB = np.array([msg.point.x, msg.point.y, msg.point.z], float)
        elif frame == self.cam_frame.lower() or frame.startswith('camera'):
            if self.T_BC is None:
                self.get_logger().warn('No base marker yet (B←C unknown).')
                return
            pC = np.array([msg.point.x, msg.point.y, msg.point.z, 1.0], float)
            pB = (self.T_BC @ pC)[:3]
        else:
            pB = np.array([msg.point.x, msg.point.y, msg.point.z], float)

        # ---- 2D IK（x,z）----
        p_des_xz = np.array([pB[0], pB[2]], float)
        theta, ok = lm_ik_2d(p_des_xz, self.L, tip_extra=self.tip_extra, theta0=np.zeros(3), iters=180, lam=1e-2)
        if not ok:
            self.get_logger().warn(f'IK not fully converged: theta={theta}')

        # ---- Δl（左＝yマイナス側で固定）----
        # Δl1 = r*θ1, Δl2 = r*(θ1+θ2), Δl3 = r*(θ1+θ2+θ3)
        dl1 = self.r_wire * (theta[0])
        dl2 = self.r_wire * (theta[0] + theta[1])
        dl3 = self.r_wire * (theta[0] + theta[1] + theta[2])
        dls = np.array([dl1, dl2, dl3], float)

        # ---- モータ角：Δα = Δl / r_pulley（rad）→ 度に変換 ----
        dalpha_rad = dls / self.r_pulley
        dalpha_deg = np.degrees(dalpha_rad)  # ★ 出力は度

        # ---- 出力角度（deg） = 初回値（deg） + Δα（deg）を index 7,8,9 に反映 ----
        cmd = self.zero_motor_deg.copy()
        for j, idx in enumerate(self.motor_idx):
            if not (0 <= idx < len(cmd)):
                self.get_logger().error(f'motor index {idx} out of range'); return
            cmd[idx] = self.zero_motor_deg[idx] + float(dalpha_deg[j])

        out = Float32MultiArray(); out.data = cmd.tolist()
        self.pub_cmd.publish(out)

        p_fk,_ = pcc_forward_end(theta, self.L, tip_extra=self.tip_extra)
        self.get_logger().info(
            f"target(B) xz=({p_des_xz[0]:.3f},{p_des_xz[1]:.3f}) "
            f"theta(deg)={[round(t*180/math.pi,2) for t in theta]} "
            f"dl(mm)={[round(x*1000,2) for x in dls]} "
            f"cmd(deg)={[round(cmd[i],3) for i in self.motor_idx]} "
            f"fk=({p_fk[0]:.3f},{p_fk[2]:.3f})"
        )

def main():
    rclpy.init()
    rclpy.spin(PCCMoveNode())
    rclpy.shutdown()

if __name__ == '__main__':
    main()
