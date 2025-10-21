#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import math
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, ColorRGBA
from geometry_msgs.msg import Point, Pose
from visualization_msgs.msg import Marker, MarkerArray
from aruco_interfaces.msg import ArucoMarkers  # ids:int32[], poses:Pose[]

# ---------- PCC forward (平面 x–z, ψは+x基準) ----------
def pcc_centerline_xy(theta, L, tip_extra=0.0, n_per_seg=50, psi0=0.0):
    xs = [0.0]; ys = [0.0]
    psi = psi0   # +X 方向を0 rad
    x = 0.0; y = 0.0
    for th, Li in zip(theta, L):
        k = th / Li if abs(Li) > 1e-12 else 0.0
        ss = np.linspace(0.0, Li, n_per_seg, endpoint=False)
        if abs(k) < 1e-10:
            for s in ss[1:]:
                xs.append(x + math.cos(psi)*s)
                ys.append(y + math.sin(psi)*s)
            x += math.cos(psi)*Li
            y += math.sin(psi)*Li
            xs.append(x); ys.append(y)
        else:
            R = 1.0/k
            for s in ss[1:]:
                psi_s = psi + k*s
                xs.append(x + R*(math.sin(psi_s) - math.sin(psi)))
                ys.append(y - R*(math.cos(psi_s) - math.cos(psi)))
            psi = psi + th
            x = xs[-1]; y = ys[-1]
    if tip_extra > 0.0:
        steps = max(2, int(n_per_seg*tip_extra/max(1e-6, sum(L))))
        for s in np.linspace(0.0, tip_extra, steps, endpoint=True)[1:]:
            xs.append(x + math.cos(psi)*s)
            ys.append(y + math.sin(psi)*s)
    return np.array(xs), np.array(ys)

# 1) 既存：拘束モデル（スラック無し）
def theta_from_motor_deg_constrained(motor_deg, zero_deg, idxs, r_wire, r_pulley):
    dalpha_deg = np.array([motor_deg[idxs[0]] - zero_deg[idxs[0]],
                           motor_deg[idxs[1]] - zero_deg[idxs[1]],
                           motor_deg[idxs[2]] - zero_deg[idxs[2]]], float)
    dalpha = np.radians(dalpha_deg)
    dls = dalpha * r_pulley
    t1 = dls[0]/r_wire
    t2 = (dls[1]-dls[0])/r_wire
    t3 = (dls[2]-dls[1])/r_wire
    return np.array([t1,t2,t3], float)

# 2) 剛性分配モデル（“全体で曲がる”）
def theta_from_motor_deg_stiffness(motor_deg, zero_deg, idxs, r_wire, r_pulley,
                                   K=(1.0,1.0,1.0), pretension=0.0):
    """
    K: 各セグの曲げ剛性(相対値でOK), pretension: 張力のしきい(簡易スラック)
    ここでは「遠位腱3のみ有効」と仮定（分配の例）。複合引きにも拡張可。
    """
    dalpha_deg = np.array([motor_deg[idxs[0]] - zero_deg[idxs[0]],
                           motor_deg[idxs[1]] - zero_deg[idxs[1]],
                           motor_deg[idxs[2]] - zero_deg[idxs[2]]], float)
    dalpha = np.radians(dalpha_deg)
    dls = dalpha * r_pulley  # [dl1, dl2, dl3]（腱の巻取り量）

    # ここでは「腱3のみが実際に張っている」ケースを例示
    dl_eff = max(0.0, float(dls[2]))  # 負はスラックとみなして0（簡易）
    if dl_eff <= pretension:
        return np.zeros(3)

    invK = np.array([1.0/K[0], 1.0/K[1], 1.0/K[2]], float)
    w = invK / invK.sum()
    thetas = (dl_eff / r_wire) * w
    return thetas


class PCCVisualizer(Node):
    def __init__(self):
        super().__init__('pcc_visualizer_node')
        # params
        self.declare_parameter('L', [0.20, 0.20, 0.20])     # m
        self.declare_parameter('tip_extra', 0.10)           # m
        self.declare_parameter('wire_radius', 0.019)        # m
        self.declare_parameter('pulley_radius', 0.008)      # m
        self.declare_parameter('motor_indices', [6,7,8])
        self.declare_parameter('base_frame', 'camera_color_optical_frame')
        self.declare_parameter('marker_ns', 'pcc_viz')
        self.declare_parameter('line_resolution', 80)
        self.declare_parameter('show_wires', False)
        self.declare_parameter('base_marker_id', 4)
        self.declare_parameter('tip_marker_id', 0)
        self.declare_parameter('camera_frame', 'camera_color_optical_frame')
        self.declare_parameter('show_measured_tip', True)
        self.declare_parameter('draw_frame', 'camera')   # 'camera' or 'base'

        self.L = np.array(self.get_parameter('L').value, float)
        self.tip_extra = float(self.get_parameter('tip_extra').value)
        self.r_wire = float(self.get_parameter('wire_radius').value)
        self.r_pulley = float(self.get_parameter('pulley_radius').value)
        self.motor_idx = list(self.get_parameter('motor_indices').value)
        self.base_frame = self.get_parameter('base_frame').value
        self.ns = self.get_parameter('marker_ns').value
        self.nseg = int(self.get_parameter('line_resolution').value)
        self.base_id = int(self.get_parameter('base_marker_id').value)
        self.tip_id = int(self.get_parameter('tip_marker_id').value)
        self.cam_frame = self.get_parameter('camera_frame').value
        self.show_meas_tip = bool(self.get_parameter('show_measured_tip').value)
        self.draw_frame = self.get_parameter('draw_frame').value

        # states
        self.zero_deg = None
        self.last_motor_deg = None
        self.T_CB = None   # C←B
        self.T_BC = None   # B←C
        self.tip_pose_C = None  # camera系のid0
        self.tip_B = None       # B系のid0
        self.base_C = None    # B系のid4

        # I/O
        self.sub_mot = self.create_subscription(
            Float32MultiArray, '/motor_current_angles', self.cb_motor, 10)
        self.sub_mark = self.create_subscription(
            ArucoMarkers, '/aruco/markers', self.cb_markers, 10)
        self.pub_mk = self.create_publisher(MarkerArray, '/pcc_visualization', 10)

        self.timer = self.create_timer(0.1, self.on_timer)
        self.get_logger().info('PCCVisualizer ready.')

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

            # base marker -> T_CB, T_BC 更新
            if self.base_id in ids:
                i = ids.index(self.base_id)
                p: Pose = msg.poses[i]
                t = np.array([p.position.x, p.position.y, p.position.z], float)
                q = p.orientation
                R = np.array([
                    [1-2*(q.y*q.y+q.z*q.z), 2*(q.x*q.y - q.z*q.w),   2*(q.x*q.z + q.y*q.w)],
                    [2*(q.x*q.y + q.z*q.w), 1-2*(q.x*q.x+q.z*q.z),   2*(q.y*q.z - q.x*q.w)],
                    [2*(q.x*q.z - q.y*q.w), 2*(q.y*q.z + q.x*q.w),   1-2*(q.x*q.x+q.y*q.y)]
                ], float)
                T_CB = np.eye(4); T_CB[:3,:3]=R; T_CB[:3,3]=t
                self.T_CB = T_CB
                Rt = R.T; tt = t
                Ti = np.eye(4); Ti[:3,:3] = Rt; Ti[:3,3] = -Rt @ tt
                self.T_BC = Ti

            # tip marker (id0)
            self.tip_pose_C = None
            self.tip_B = None
            self.base_C = t.copy()
            if self.tip_id in ids:
                j = ids.index(self.tip_id)
                tip_pose = msg.poses[j]      # camera系
                self.tip_pose_C = tip_pose
                if self.T_BC is not None:
                    pC = np.array([tip_pose.position.x, tip_pose.position.y, tip_pose.position.z, 1.0], float)
                    pB = (self.T_BC @ pC)[:3]
                    self.tip_B = pB

        except Exception as e:
            self.get_logger().warn(f'/aruco/markers parse error: {e}')

    def on_timer(self):
        if self.last_motor_deg is None or self.zero_deg is None:
            return
        if self.base_C is None:
            # ベースの位置が未検出なら描けない
            return

        # θ 推定（従来どおり）
        theta = theta_from_motor_deg_constrained(self.last_motor_deg, self.zero_deg,
                   self.motor_idx, self.r_wire, self.r_pulley)
        # theta = theta_from_motor_deg_stiffness(self.last_motor_deg, self.zero_deg,
        #                             self.motor_idx, self.r_wire, self.r_pulley)
        
        psi0 = math.pi

        # XY 平面で +X スタート（psi0=0）
        X, Y = pcc_centerline_xy(theta, self.L, tip_extra=self.tip_extra, n_per_seg=self.nseg, psi0=psi0)

        # 始点（camera系）に載せる： [x0+x, y0+y, z0]
        x0, y0, z0 = float(self.base_C[0]), float(self.base_C[1]), float(self.base_C[2])
        pts = [(x0 + float(x), y0 + float(y), z0) for x, y in zip(X, Y)]
        tip_pos = (pts[-1][0], pts[-1][1], pts[-1][2])

        # ---- MarkerArray（camera_color_optical_frame に直接）----
        header_frame = self.cam_frame  # 'camera_color_optical_frame'
        ma = MarkerArray()

        line = Marker()
        line.header.frame_id = header_frame
        line.header.stamp = self.get_clock().now().to_msg()
        line.ns = self.ns; line.id = 1
        line.type = Marker.LINE_STRIP
        line.action = Marker.ADD
        line.scale.x = 0.004
        line.color = ColorRGBA(r=0.1, g=0.7, b=1.0, a=1.0)
        line.points = [Point(x=px, y=py, z=pz) for (px,py,pz) in pts]
        ma.markers.append(line)

        tip = Marker()
        tip.header.frame_id = header_frame
        tip.header.stamp = line.header.stamp
        tip.ns = self.ns; tip.id = 2
        tip.type = Marker.SPHERE
        tip.action = Marker.ADD
        tip.scale.x = tip.scale.y = tip.scale.z = 0.01
        tip.color = ColorRGBA(r=1.0, g=0.2, b=0.2, a=1.0)
        tip.pose.position.x, tip.pose.position.y, tip.pose.position.z = tip_pos
        ma.markers.append(tip)

        # 実測先端（id=0）も camera系のまま重ねる
        if self.show_meas_tip and self.tip_pose_C is not None:
            mt = Marker()
            mt.header.frame_id = header_frame
            mt.header.stamp = line.header.stamp
            mt.ns = self.ns; mt.id = 3
            mt.type = Marker.SPHERE
            mt.action = Marker.ADD
            mt.scale.x = mt.scale.y = mt.scale.z = 0.012
            mt.color = ColorRGBA(r=0.2, g=1.0, b=0.2, a=0.9)
            mt.pose = self.tip_pose_C
            ma.markers.append(mt)

        self.pub_mk.publish(ma)


def main():
    rclpy.init()
    rclpy.spin(PCCVisualizer())
    rclpy.shutdown()

if __name__ == '__main__':
    main()
