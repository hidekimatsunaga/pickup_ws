#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
from dataclasses import dataclass
from typing import Callable, List, Optional, Tuple

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist, PointStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool


def clamp(x: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, x))


def wrap_pi(a: float) -> float:
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def yaw_from_quat(q) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


@dataclass(frozen=True)
class Pattern:
    name: str
    duration_s: float
    generator: Callable[[float], Tuple[float, float, float]]  # (vx, vy, wz) in base frame
    yaw_hold: bool = False
    center_xy: Optional[Tuple[float, float]] = None


class MovementGuruguruShowcase(Node):
    def __init__(self) -> None:
        super().__init__('movement_guruguru_showcase')

        # ---- topics ----
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('enable_topic', '/move_test/enable')
        self.declare_parameter('odom_topic', '/wheel_odom')

        # trajectory logger topics (optional)
        self.declare_parameter('goal_topic', '/traj/goal')
        self.declare_parameter('center_topic', '/traj/center')

        # ---- behavior ----
        self.declare_parameter('loop', True)
        self.declare_parameter('settle_time_s', 0.20)

        # safety clamps
        self.declare_parameter('v_max', 0.6)   # [m/s] clamp for |vx|,|vy|
        self.declare_parameter('wz_max', 1.6)  # [rad/s]

        # yaw hold (used only for patterns with yaw_hold=True)
        self.declare_parameter('k_yaw', 2.0)

        # pattern knobs
        self.declare_parameter('dur_rotate_s', 3.0)
        self.declare_parameter('dur_circle_s', 7.0)
        self.declare_parameter('dur_strafe_circle_s', 7.0)
        self.declare_parameter('dur_figure8_s', 9.0)
        self.declare_parameter('dur_spiral_s', 9.0)
        self.declare_parameter('dur_zigzag_s', 7.0)

        self.declare_parameter('rotate_wz', 1.1)

        self.declare_parameter('circle_vx', 0.25)
        self.declare_parameter('circle_wz', 0.55)

        # holonomic circle: vx=v*cos(wt), vy=v*sin(wt)
        self.declare_parameter('strafe_circle_v', 0.35)
        self.declare_parameter('strafe_circle_omega', 0.70)

        # unicycle-ish figure8: vx constant, wz = A*sin(wt)
        self.declare_parameter('figure8_vx', 0.30)
        self.declare_parameter('figure8_wz_amp', 1.0)
        self.declare_parameter('figure8_omega', 0.85)

        # spiral: vx = (v0 + a*t)*cos(wt), vy = (v0 + a*t)*sin(wt)
        self.declare_parameter('spiral_v0', 0.15)
        self.declare_parameter('spiral_acc', 0.04)  # [m/s^2] speed increase
        self.declare_parameter('spiral_omega', 0.85)

        # zigzag: lateral square wave + small forward
        self.declare_parameter('zigzag_vx', 0.12)
        self.declare_parameter('zigzag_vy', 0.40)
        self.declare_parameter('zigzag_period_s', 1.2)

        # ---- load params ----
        self.loop = bool(self.get_parameter('loop').value)
        self.settle_time_s = float(self.get_parameter('settle_time_s').value)

        self.v_max = float(self.get_parameter('v_max').value)
        self.wz_max = float(self.get_parameter('wz_max').value)
        self.k_yaw = float(self.get_parameter('k_yaw').value)

        # pattern durations
        dur_rotate_s = float(self.get_parameter('dur_rotate_s').value)
        dur_circle_s = float(self.get_parameter('dur_circle_s').value)
        dur_strafe_circle_s = float(self.get_parameter('dur_strafe_circle_s').value)
        dur_figure8_s = float(self.get_parameter('dur_figure8_s').value)
        dur_spiral_s = float(self.get_parameter('dur_spiral_s').value)
        dur_zigzag_s = float(self.get_parameter('dur_zigzag_s').value)

        rotate_wz = float(self.get_parameter('rotate_wz').value)

        circle_vx = float(self.get_parameter('circle_vx').value)
        circle_wz = float(self.get_parameter('circle_wz').value)

        strafe_circle_v = float(self.get_parameter('strafe_circle_v').value)
        strafe_circle_omega = float(self.get_parameter('strafe_circle_omega').value)

        figure8_vx = float(self.get_parameter('figure8_vx').value)
        figure8_wz_amp = float(self.get_parameter('figure8_wz_amp').value)
        figure8_omega = float(self.get_parameter('figure8_omega').value)

        spiral_v0 = float(self.get_parameter('spiral_v0').value)
        spiral_acc = float(self.get_parameter('spiral_acc').value)
        spiral_omega = float(self.get_parameter('spiral_omega').value)

        zigzag_vx = float(self.get_parameter('zigzag_vx').value)
        zigzag_vy = float(self.get_parameter('zigzag_vy').value)
        zigzag_period_s = float(self.get_parameter('zigzag_period_s').value)

        # ---- pubs/subs ----
        cmd_vel_topic = str(self.get_parameter('cmd_vel_topic').value)
        enable_topic = str(self.get_parameter('enable_topic').value)
        odom_topic = str(self.get_parameter('odom_topic').value)
        goal_topic = str(self.get_parameter('goal_topic').value)
        center_topic = str(self.get_parameter('center_topic').value)

        self.pub_cmd = self.create_publisher(Twist, cmd_vel_topic, 10)
        self.pub_goal = self.create_publisher(PointStamped, goal_topic, 10)
        self.pub_center = self.create_publisher(PointStamped, center_topic, 10)

        self.sub_enable = self.create_subscription(Bool, enable_topic, self.cb_enable, 10)
        self.sub_odom = self.create_subscription(Odometry, odom_topic, self.cb_odom, 20)

        # ---- state ----
        self.enabled = False
        self.odom: Optional[Odometry] = None
        self.yaw0: Optional[float] = None

        self.patterns: List[Pattern] = self._build_patterns(
            dur_rotate_s=dur_rotate_s,
            dur_circle_s=dur_circle_s,
            dur_strafe_circle_s=dur_strafe_circle_s,
            dur_figure8_s=dur_figure8_s,
            dur_spiral_s=dur_spiral_s,
            dur_zigzag_s=dur_zigzag_s,
            rotate_wz=rotate_wz,
            circle_vx=circle_vx,
            circle_wz=circle_wz,
            strafe_circle_v=strafe_circle_v,
            strafe_circle_omega=strafe_circle_omega,
            figure8_vx=figure8_vx,
            figure8_wz_amp=figure8_wz_amp,
            figure8_omega=figure8_omega,
            spiral_v0=spiral_v0,
            spiral_acc=spiral_acc,
            spiral_omega=spiral_omega,
            zigzag_vx=zigzag_vx,
            zigzag_vy=zigzag_vy,
            zigzag_period_s=zigzag_period_s,
        )

        self.idx = 0
        self.stage = 'IDLE'  # IDLE | RUN | SETTLE
        self.stage_t0 = None

        self.timer = self.create_timer(0.02, self.on_timer)  # 50Hz

        self.get_logger().info(
            f"Guruguru showcase ready. cmd_vel={cmd_vel_topic} enable={enable_topic} odom={odom_topic}"
        )
        self.get_logger().info('Publish enable=true to start / enable=false to stop.')

    def _build_patterns(
        self,
        *,
        dur_rotate_s: float,
        dur_circle_s: float,
        dur_strafe_circle_s: float,
        dur_figure8_s: float,
        dur_spiral_s: float,
        dur_zigzag_s: float,
        rotate_wz: float,
        circle_vx: float,
        circle_wz: float,
        strafe_circle_v: float,
        strafe_circle_omega: float,
        figure8_vx: float,
        figure8_wz_amp: float,
        figure8_omega: float,
        spiral_v0: float,
        spiral_acc: float,
        spiral_omega: float,
        zigzag_vx: float,
        zigzag_vy: float,
        zigzag_period_s: float,
    ) -> List[Pattern]:
        patterns: List[Pattern] = []

        patterns.append(
            Pattern(
                name='rotate_in_place',
                duration_s=max(0.0, dur_rotate_s),
                generator=lambda t: (0.0, 0.0, rotate_wz),
                yaw_hold=False,
            )
        )

        patterns.append(
            Pattern(
                name='circle_unicycle',
                duration_s=max(0.0, dur_circle_s),
                generator=lambda t: (circle_vx, 0.0, circle_wz),
                yaw_hold=False,
            )
        )

        patterns.append(
            Pattern(
                name='strafe_circle_holonomic',
                duration_s=max(0.0, dur_strafe_circle_s),
                generator=lambda t: (
                    strafe_circle_v * math.cos(strafe_circle_omega * t),
                    strafe_circle_v * math.sin(strafe_circle_omega * t),
                    0.0,
                ),
                yaw_hold=True,
                center_xy=(0.0, 0.0),
            )
        )

        patterns.append(
            Pattern(
                name='figure8_wiggle',
                duration_s=max(0.0, dur_figure8_s),
                generator=lambda t: (figure8_vx, 0.0, figure8_wz_amp * math.sin(figure8_omega * t)),
                yaw_hold=False,
            )
        )

        patterns.append(
            Pattern(
                name='spiral_holonomic',
                duration_s=max(0.0, dur_spiral_s),
                generator=lambda t: (
                    (spiral_v0 + spiral_acc * t) * math.cos(spiral_omega * t),
                    (spiral_v0 + spiral_acc * t) * math.sin(spiral_omega * t),
                    0.0,
                ),
                yaw_hold=True,
                center_xy=(0.0, 0.0),
            )
        )

        def zigzag_gen(t: float) -> Tuple[float, float, float]:
            if zigzag_period_s <= 1e-6:
                sign = 1.0
            else:
                phase = (t % zigzag_period_s) / zigzag_period_s
                sign = 1.0 if phase < 0.5 else -1.0
            return (zigzag_vx, sign * zigzag_vy, 0.0)

        patterns.append(
            Pattern(
                name='zigzag_strafe',
                duration_s=max(0.0, dur_zigzag_s),
                generator=zigzag_gen,
                yaw_hold=True,
            )
        )

        # filter out zero-duration patterns
        return [p for p in patterns if p.duration_s > 1e-3]

    def cb_odom(self, msg: Odometry) -> None:
        self.odom = msg

    def cb_enable(self, msg: Bool) -> None:
        self.enabled = bool(msg.data)
        if self.enabled:
            self.idx = 0
            self.stage = 'RUN'
            self.stage_t0 = self.now_s()
            self.capture_yaw0_if_possible()
            self.publish_markers_for_current_pattern()
            self.get_logger().info(f'Start: {self.patterns[self.idx].name if self.patterns else "(no patterns)"}')
        else:
            self.publish_stop()
            self.stage = 'IDLE'
            self.stage_t0 = None
            self.yaw0 = None

    def now_s(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def capture_yaw0_if_possible(self) -> None:
        if self.odom is None:
            self.yaw0 = None
            return
        self.yaw0 = yaw_from_quat(self.odom.pose.pose.orientation)

    def publish_stop(self) -> None:
        self.pub_cmd.publish(Twist())

    def publish_markers_for_current_pattern(self) -> None:
        if not self.patterns:
            return
        pat = self.patterns[self.idx]

        # center marker (optional)
        if pat.center_xy is not None:
            cx, cy = pat.center_xy
            msg = PointStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'odom'
            msg.point.x = float(cx)
            msg.point.y = float(cy)
            msg.point.z = 0.0
            self.pub_center.publish(msg)

        # goal marker: publish a rough reference point (not used for control)
        # For circles: a point on the circle; otherwise NaN-ish sentinel is not allowed in ROS msg,
        # so we simply omit if not meaningful.
        if pat.name in ('circle_unicycle', 'strafe_circle_holonomic'):
            # nominal radius estimate: r = v/|w|
            # (for strafe circle, r = v/omega)
            if pat.name == 'circle_unicycle':
                v_nom = abs(float(self.get_parameter('circle_vx').value))
                w_nom = abs(float(self.get_parameter('circle_wz').value))
                r = v_nom / max(1e-6, w_nom)
            else:
                v_nom = abs(float(self.get_parameter('strafe_circle_v').value))
                w_nom = abs(float(self.get_parameter('strafe_circle_omega').value))
                r = v_nom / max(1e-6, w_nom)
            msg = PointStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'odom'
            msg.point.x = float(r)
            msg.point.y = 0.0
            msg.point.z = 0.0
            self.pub_goal.publish(msg)

    def apply_yaw_hold(self, wz_cmd: float) -> float:
        if self.yaw0 is None or self.odom is None:
            return wz_cmd
        yaw = yaw_from_quat(self.odom.pose.pose.orientation)
        yaw_err = wrap_pi(self.yaw0 - yaw)
        return clamp(wz_cmd + self.k_yaw * yaw_err, -self.wz_max, self.wz_max)

    def next_pattern(self) -> None:
        if not self.patterns:
            self.enabled = False
            self.stage = 'IDLE'
            self.publish_stop()
            return

        self.idx += 1
        if self.idx >= len(self.patterns):
            if self.loop:
                self.idx = 0
            else:
                self.get_logger().info('Done (one cycle).')
                self.enabled = False
                self.stage = 'IDLE'
                self.publish_stop()
                return

        self.stage = 'RUN'
        self.stage_t0 = self.now_s()
        self.capture_yaw0_if_possible()
        self.publish_markers_for_current_pattern()
        self.get_logger().info(f'Next: {self.patterns[self.idx].name}')

    def on_timer(self) -> None:
        if not self.enabled or self.stage == 'IDLE':
            return
        if not self.patterns:
            self.publish_stop()
            self.enabled = False
            self.stage = 'IDLE'
            self.get_logger().warn('No patterns configured. Stopping.')
            return

        now = self.now_s()
        if self.stage_t0 is None:
            self.stage_t0 = now
        t = now - self.stage_t0

        if self.stage == 'SETTLE':
            if t >= self.settle_time_s:
                self.next_pattern()
            else:
                self.publish_stop()
            return

        # RUN
        pat = self.patterns[self.idx]
        if t >= pat.duration_s:
            self.publish_stop()
            self.stage = 'SETTLE'
            self.stage_t0 = now
            return

        vx, vy, wz = pat.generator(t)

        vx = clamp(vx, -self.v_max, self.v_max)
        vy = clamp(vy, -self.v_max, self.v_max)
        wz = clamp(wz, -self.wz_max, self.wz_max)

        if pat.yaw_hold:
            wz = self.apply_yaw_hold(wz)

        cmd = Twist()
        cmd.linear.x = float(vx)
        cmd.linear.y = float(vy)
        cmd.angular.z = float(wz)
        self.pub_cmd.publish(cmd)


def main() -> None:
    rclpy.init()
    node = MovementGuruguruShowcase()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.publish_stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
