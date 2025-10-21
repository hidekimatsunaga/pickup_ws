#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from builtin_interfaces.msg import Time
from rcl_interfaces.msg import ParameterDescriptor, ParameterType

def _now(node: Node) -> Time:
    return node.get_clock().now().to_msg()

class PCCTargetPublisher(Node):
    """
    /pcc_target に PointStamped を順次 publish して実験できるノード。

    モード:
      - mode='sequence' : 任意の点列を順に送る（points_xyz_flat で与える）
      - mode='sweep'    : 1軸スイープ（center を基準に axis を start→stop）
      - mode='grid'     : 2軸グリッド（center の周りに格子状）

    各点は dwell_sec 秒キープ。その間 publish_hz で連続送信（安定のため）。
    端までいったら loop=true なら繰り返す。
    """

    def __init__(self):
        super().__init__('pcc_target_publisher')

        # ---- パラメータ宣言（型を最初に確定！）----
        self.declare_parameter('frame_id', 'base')
        self.declare_parameter('mode', 'sequence')         # 'sequence' | 'sweep' | 'grid'
        self.declare_parameter('publish_hz', 10.0)
        self.declare_parameter('dwell_sec', 1.0)
        self.declare_parameter('loop', True)

        # DOUBLE_ARRAY で明示（★ここ重要：一度だけ宣言）
        double_array_desc = ParameterDescriptor(type=ParameterType.PARAMETER_DOUBLE_ARRAY)
        self.declare_parameter('points_xyz_flat', [0.0,0.0,0.0], descriptor=double_array_desc)

        # sweep 用
        self.declare_parameter('center', [0.20, 0.00, 0.30])  # [x,y,z]
        self.declare_parameter('axis', 'x')                   # 'x' | 'y' | 'z'
        self.declare_parameter('start', -0.02)
        self.declare_parameter('stop', 0.02)
        self.declare_parameter('steps', 5)

        # grid 用
        self.declare_parameter('grid_axis1', 'x')
        self.declare_parameter('grid_axis2', 'z')
        self.declare_parameter('grid_span1', 0.04)
        self.declare_parameter('grid_span2', 0.04)
        self.declare_parameter('grid_steps1', 5)
        self.declare_parameter('grid_steps2', 5)
        self.declare_parameter('grid_snake', True)

        # ---- 取得 ----
        self.frame_id   = self.get_parameter('frame_id').value
        self.mode       = self.get_parameter('mode').value.lower()
        self.publish_hz = float(self.get_parameter('publish_hz').value)
        self.dwell_sec  = float(self.get_parameter('dwell_sec').value)
        self.loop       = bool(self.get_parameter('loop').value)

        self.pub = self.create_publisher(PointStamped, '/pcc_target', 10)

        # 点列生成
        self.points = self._build_points()
        if not self.points:
            self.get_logger().error('No points to publish. Check parameters.')
            self.points = [(0.0, 0.0, 0.0)]

        # 実行状態
        self.idx = 0
        self.last_switch_time = self.get_clock().now()

        # タイマ（連続publish & dwell管理）
        period = max(1e-3, 1.0 / max(1e-6, self.publish_hz))
        self.timer = self.create_timer(period, self._on_timer)

        self.get_logger().info(
            f"PCCTargetSweeper started: mode={self.mode}, frame='{self.frame_id}', "
            f"{len(self.points)} pts, dwell={self.dwell_sec}s, hz={self.publish_hz}, loop={self.loop}"
        )

    # ---------- 点列生成 ----------
    def _build_points(self):
        if self.mode == 'sequence':
            raw = list(self.get_parameter('points_xyz_flat').value or [])
            if len(raw) % 3 != 0:
                self.get_logger().warn('points_xyz_flat length is not multiple of 3. Truncating.')
                raw = raw[:len(raw)//3*3]
            pts = [(raw[i], raw[i+1], raw[i+2]) for i in range(0, len(raw), 3)]
            return pts

        elif self.mode == 'sweep':
            center = list(self.get_parameter('center').value)
            axis   = self.get_parameter('axis').value.lower()
            start  = float(self.get_parameter('start').value)
            stop   = float(self.get_parameter('stop').value)
            steps  = int(self.get_parameter('steps').value)
            if steps < 2:
                steps = 2
            vals = [start + (stop-start)*i/(steps-1) for i in range(steps)]
            axis_idx = {'x':0,'y':1,'z':2}.get(axis, 0)
            pts = []
            for v in vals:
                p = center.copy()
                p[axis_idx] = center[axis_idx] + v
                pts.append(tuple(p))
            return pts

        elif self.mode == 'grid':
            center = list(self.get_parameter('center').value)
            a1 = self.get_parameter('grid_axis1').value.lower()
            a2 = self.get_parameter('grid_axis2').value.lower()
            span1 = float(self.get_parameter('grid_span1').value)
            span2 = float(self.get_parameter('grid_span2').value)
            n1 = max(2, int(self.get_parameter('grid_steps1').value))
            n2 = max(2, int(self.get_parameter('grid_steps2').value))
            snake = bool(self.get_parameter('grid_snake').value)

            idx1 = {'x':0,'y':1,'z':2}.get(a1, 0)
            idx2 = {'x':0,'y':1,'z':2}.get(a2, 2)
            vals1 = [(-span1*0.5) + (span1)*(i/(n1-1)) for i in range(n1)]
            vals2 = [(-span2*0.5) + (span2)*(j/(n2-1)) for j in range(n2)]

            pts = []
            for j, v2 in enumerate(vals2):
                row = []
                for i, v1 in enumerate(vals1):
                    p = center.copy()
                    p[idx1] = center[idx1] + v1
                    p[idx2] = center[idx2] + v2
                    row.append(tuple(p))
                if snake and (j % 2 == 1):
                    row = list(reversed(row))
                pts.extend(row)
            return pts

        else:
            self.get_logger().warn(f"Unknown mode '{self.mode}', fallback to sequence with current center")
            p = self.get_parameter('center').value or [0.0,0.0,0.0]
            return [tuple(p)]

    # ---------- 実行 ----------
    def _publish_point(self, p):
        msg = PointStamped()
        msg.header.frame_id = self.frame_id
        msg.header.stamp = _now(self)
        msg.point.x, msg.point.y, msg.point.z = float(p[0]), float(p[1]), float(p[2])
        self.pub.publish(msg)

    def _on_timer(self):
        # 現在の点を連続送信
        p = self.points[self.idx]
        self._publish_point(p)

        # dwell を過ぎたら次の点へ
        now = self.get_clock().now()
        if (now - self.last_switch_time).nanoseconds * 1e-9 >= self.dwell_sec:
            self.idx += 1
            if self.idx >= len(self.points):
                if self.loop:
                    self.idx = 0
                else:
                    self.idx = len(self.points) - 1
                    self.get_logger().info('Reached last point (loop=False). Holding last target.')
                    self.last_switch_time = now
                    return
            self.last_switch_time = now
            new_p = self.points[self.idx]   # ★新しい点でログ
            self.get_logger().info(
                f"Switch to point {self.idx+1}/{len(self.points)}: "
                f"x={new_p[0]:.3f}, y={new_p[1]:.3f}, z={new_p[2]:.3f} in '{self.frame_id}'"
            )

def main():
    rclpy.init()
    rclpy.spin(PCCTargetPublisher())
    rclpy.shutdown()

if __name__ == '__main__':
    main()
