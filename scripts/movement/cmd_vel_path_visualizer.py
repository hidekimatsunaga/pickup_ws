#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
from typing import Optional

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Point, PoseStamped, Twist
from nav_msgs.msg import Path
from std_msgs.msg import Bool
from visualization_msgs.msg import Marker


def wrap_pi(a: float) -> float:
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def quat_from_yaw(yaw: float):
    # geometry_msgs/Quaternion
    # yaw-only: q = [0,0,sin(y/2),cos(y/2)]
    from geometry_msgs.msg import Quaternion

    q = Quaternion()
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(yaw * 0.5)
    q.w = math.cos(yaw * 0.5)
    return q


class CmdVelPathVisualizer(Node):
    """Visualize integrated trajectory from /cmd_vel.

    Notes:
      - This is dead-reckoning integration (no slip, no odom fusion).
      - Useful for checking what trajectory your cmd stream implies.
      - View in RViz2 by adding displays for the published Path/Marker topics.
    """

    def __init__(self) -> None:
        super().__init__('cmd_vel_path_visualizer')

        # ---- params ----
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('enable_topic', '/move_test/enable')

        self.declare_parameter('frame_id', 'odom')
        self.declare_parameter('path_topic', '/cmd_vel/path')
        self.declare_parameter('marker_topic', '/cmd_vel/path_marker')
        self.declare_parameter('heading_marker_topic', '/cmd_vel/heading_marker')

        self.declare_parameter('publish_hz', 10.0)

        self.declare_parameter('omega_eps', 0.02)
        self.declare_parameter('v_eps', 0.001)
        self.declare_parameter('max_dt', 1.0)

        self.declare_parameter('max_points', 4000)
        self.declare_parameter('downsample_n', 1)  # append one point per N publish cycles

        # marker style
        self.declare_parameter('line_width', 0.03)
        self.declare_parameter('color_r', 0.1)
        self.declare_parameter('color_g', 0.9)
        self.declare_parameter('color_b', 0.2)
        self.declare_parameter('color_a', 0.9)
        self.declare_parameter('heading_length', 0.25)
        self.declare_parameter('heading_width', 0.05)
        self.declare_parameter('heading_r', 0.9)
        self.declare_parameter('heading_g', 0.2)
        self.declare_parameter('heading_b', 0.1)
        self.declare_parameter('heading_a', 0.9)

        self.cmd_vel_topic = str(self.get_parameter('cmd_vel_topic').value)
        self.enable_topic = str(self.get_parameter('enable_topic').value)
        self.frame_id = str(self.get_parameter('frame_id').value)
        self.path_topic = str(self.get_parameter('path_topic').value)
        self.marker_topic = str(self.get_parameter('marker_topic').value)
        self.heading_marker_topic = str(self.get_parameter('heading_marker_topic').value)

        self.publish_hz = float(self.get_parameter('publish_hz').value)

        self.omega_eps = float(self.get_parameter('omega_eps').value)
        self.v_eps = float(self.get_parameter('v_eps').value)
        self.max_dt = float(self.get_parameter('max_dt').value)

        self.max_points = int(self.get_parameter('max_points').value)
        self.downsample_n = max(1, int(self.get_parameter('downsample_n').value))

        self.line_width = float(self.get_parameter('line_width').value)
        self.color_r = float(self.get_parameter('color_r').value)
        self.color_g = float(self.get_parameter('color_g').value)
        self.color_b = float(self.get_parameter('color_b').value)
        self.color_a = float(self.get_parameter('color_a').value)
        self.heading_length = float(self.get_parameter('heading_length').value)
        self.heading_width = float(self.get_parameter('heading_width').value)
        self.heading_r = float(self.get_parameter('heading_r').value)
        self.heading_g = float(self.get_parameter('heading_g').value)
        self.heading_b = float(self.get_parameter('heading_b').value)
        self.heading_a = float(self.get_parameter('heading_a').value)

        # ---- pubs/subs ----
        self.sub_cmd = self.create_subscription(Twist, self.cmd_vel_topic, self.cb_cmd, 20)
        self.sub_enable = self.create_subscription(Bool, self.enable_topic, self.cb_enable, 10)

        self.pub_path = self.create_publisher(Path, self.path_topic, 10)
        self.pub_marker = self.create_publisher(Marker, self.marker_topic, 10)
        self.pub_heading = self.create_publisher(Marker, self.heading_marker_topic, 10)

        # ---- state ----
        self.enabled = False
        self.latest_twist = Twist()
        self.last_time: Optional[float] = None

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        self.path_msg = Path()
        self.path_msg.header.frame_id = self.frame_id

        self.marker_msg = self._make_marker_template()
        self.heading_marker_msg = self._make_heading_marker_template()

        self.pub_cycle = 0

        # integrate at 50Hz; publish at publish_hz
        self.timer_int = self.create_timer(0.02, self.on_integrate_timer)
        self.timer_pub = self.create_timer(1.0 / max(1e-3, self.publish_hz), self.on_publish_timer)

        self.get_logger().info(
            f"CmdVelPathVisualizer ready. cmd_vel={self.cmd_vel_topic} enable={self.enable_topic}"
        )
        self.get_logger().info(
            f"Publishing: Path={self.path_topic}, Marker={self.marker_topic}, Heading={self.heading_marker_topic} (frame_id={self.frame_id})"
        )

    def _make_marker_template(self) -> Marker:
        m = Marker()
        m.header.frame_id = self.frame_id
        m.ns = 'cmd_vel_path'
        m.id = 0
        m.type = Marker.LINE_STRIP
        m.action = Marker.ADD
        m.pose.orientation.w = 1.0
        m.scale.x = float(self.line_width)
        m.color.r = float(self.color_r)
        m.color.g = float(self.color_g)
        m.color.b = float(self.color_b)
        m.color.a = float(self.color_a)
        m.lifetime.sec = 0
        m.lifetime.nanosec = 0
        return m

    def _make_heading_marker_template(self) -> Marker:
        m = Marker()
        m.header.frame_id = self.frame_id
        m.ns = 'cmd_vel_heading'
        m.id = 1
        m.type = Marker.LINE_LIST
        m.action = Marker.ADD
        m.pose.orientation.w = 1.0
        m.scale.x = float(self.heading_width)
        m.color.r = float(self.heading_r)
        m.color.g = float(self.heading_g)
        m.color.b = float(self.heading_b)
        m.color.a = float(self.heading_a)
        m.lifetime.sec = 0
        m.lifetime.nanosec = 0
        return m

    def cb_cmd(self, msg: Twist) -> None:
        self.latest_twist = msg

    def cb_enable(self, msg: Bool) -> None:
        new_enabled = bool(msg.data)
        if new_enabled and not self.enabled:
            self.reset_trajectory()
            self.get_logger().info('Enabled: reset trajectory origin.')
        if (not new_enabled) and self.enabled:
            self.get_logger().info('Disabled: stop integrating (path retained).')
        self.enabled = new_enabled

    def now_s(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def reset_trajectory(self) -> None:
        self.last_time = None
        self.latest_twist = Twist()
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        self.path_msg = Path()
        self.path_msg.header.frame_id = self.frame_id

        self.marker_msg = self._make_marker_template()
        self.marker_msg.points.clear()

        self.heading_marker_msg = self._make_heading_marker_template()
        self.heading_marker_msg.points.clear()

        self.pub_cycle = 0

    def _apply_deadzone(self, v: float, eps: float) -> float:
        return 0.0 if abs(v) < eps else v

    def on_integrate_timer(self) -> None:
        if not self.enabled:
            return

        now = self.now_s()
        if self.last_time is None:
            self.last_time = now
            return

        dt = now - self.last_time
        self.last_time = now

        if dt <= 0.0 or dt > self.max_dt:
            return

        vbx = self._apply_deadzone(self.latest_twist.linear.x, self.v_eps)
        vby = self._apply_deadzone(self.latest_twist.linear.y, self.v_eps)
        omega = self._apply_deadzone(self.latest_twist.angular.z, self.omega_eps)

        # integrate orientation
        self.yaw = wrap_pi(self.yaw + omega * dt)

        # rotate base velocity to world
        c = math.cos(self.yaw)
        s = math.sin(self.yaw)
        vwx = c * vbx - s * vby
        vwy = s * vbx + c * vby

        self.x += vwx * dt
        self.y += vwy * dt

    def _append_point(self) -> None:
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = self.frame_id
        pose.pose.position.x = float(self.x)
        pose.pose.position.y = float(self.y)
        pose.pose.position.z = 0.0
        pose.pose.orientation = quat_from_yaw(self.yaw)

        self.path_msg.poses.append(pose)

        p = Point()
        p.x = float(self.x)
        p.y = float(self.y)
        p.z = 0.0
        self.marker_msg.points.append(p)

        if len(self.path_msg.poses) > self.max_points:
            self.path_msg.poses.pop(0)
        if len(self.marker_msg.points) > self.max_points:
            self.marker_msg.points.pop(0)

        # heading line: two points forming a small segment in yaw direction
        hx = self.x + self.heading_length * math.cos(self.yaw)
        hy = self.y + self.heading_length * math.sin(self.yaw)
        p0 = Point()
        p0.x = float(self.x)
        p0.y = float(self.y)
        p0.z = 0.0
        p1 = Point()
        p1.x = float(hx)
        p1.y = float(hy)
        p1.z = 0.0
        self.heading_marker_msg.points.append(p0)
        self.heading_marker_msg.points.append(p1)
        if len(self.heading_marker_msg.points) > 2 * self.max_points:
            self.heading_marker_msg.points = self.heading_marker_msg.points[2:]

    def on_publish_timer(self) -> None:
        # publish even if disabled (to keep RViz visible), but do not append points unless enabled
        stamp = self.get_clock().now().to_msg()

        if self.enabled:
            self.pub_cycle += 1
            if (self.pub_cycle % self.downsample_n) == 0:
                self._append_point()

        self.path_msg.header.stamp = stamp
        self.marker_msg.header.stamp = stamp
        self.heading_marker_msg.header.stamp = stamp

        self.pub_path.publish(self.path_msg)
        self.pub_marker.publish(self.marker_msg)
        self.pub_heading.publish(self.heading_marker_msg)


def main() -> None:
    rclpy.init()
    node = CmdVelPathVisualizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
