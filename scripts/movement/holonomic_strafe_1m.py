#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from nav_msgs.msg import Odometry

def yaw_from_quat(q) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y*q.y + q.z*q.z)
    return math.atan2(siny_cosp, cosy_cosp)

def wrap_pi(a):
    return (a + math.pi) % (2.0*math.pi) - math.pi

def clamp(x, lo, hi):
    return max(lo, min(hi, x))

class HolonomicStrafe1m(Node):
    """
    Pure strafe for holonomic base:
      - keep yaw (optional small correction using /odom)
      - move lateral (linear.y) until lateral displacement reaches 1m
    """

    def __init__(self):
        super().__init__('holonomic_strafe_1m')

        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        # In this workspace, robot odometry is typically published as /wheel_odom
        # (from robot_motor2/robot_odom_node). Override with --ros-args -p odom_topic:=/odom if needed.
        self.declare_parameter('odom_topic', '/wheel_odom')
        self.declare_parameter('enable_topic', '/move_test/enable')

        self.declare_parameter('distance_y', 1.0)   # [m] lateral move
        self.declare_parameter('vy', 0.25)          # [m/s]
        self.declare_parameter('k_yaw', 2.0)        # yaw hold gain
        self.declare_parameter('wz_max', 1.0)       # [rad/s]
        self.declare_parameter('pos_tol', 0.02)     # [m]
        # If odom is unavailable, optionally run open-loop using time
        self.declare_parameter('open_loop_on_no_odom', True)

        self.distance_y = float(self.get_parameter('distance_y').value)
        self.vy = float(self.get_parameter('vy').value)
        self.k_yaw = float(self.get_parameter('k_yaw').value)
        self.wz_max = float(self.get_parameter('wz_max').value)
        self.pos_tol = float(self.get_parameter('pos_tol').value)
        self.open_loop_on_no_odom = bool(self.get_parameter('open_loop_on_no_odom').value)

        self.enabled = False
        self.odom = None
        self.yaw0 = None
        self.x0 = None
        self.y0 = None
        # Open-loop time mode state
        self.t0 = None
        self.duration = None

        odom_topic = str(self.get_parameter('odom_topic').value)
        enable_topic = str(self.get_parameter('enable_topic').value)
        cmd_vel_topic = str(self.get_parameter('cmd_vel_topic').value)

        self.sub_odom = self.create_subscription(Odometry, odom_topic, self.cb_odom, 20)
        self.sub_en = self.create_subscription(Bool, enable_topic, self.cb_enable, 10)
        self.pub = self.create_publisher(Twist, cmd_vel_topic, 10)

        self.timer = self.create_timer(0.02, self.on_timer)  # 50 Hz
        self.get_logger().info(
            f"HolonomicStrafe1m ready. cmd_vel={cmd_vel_topic} odom={odom_topic} enable={enable_topic}"
        )
        self.get_logger().info("Publish enable=true to start.")

    def cb_odom(self, msg: Odometry):
        self.odom = msg

    def cb_enable(self, msg: Bool):
        self.enabled = bool(msg.data)
        if self.enabled:
            # If odom is present, capture start as usual.
            if self.odom is not None:
                self.capture_start()
            else:
                # Fallback to open-loop time-based move if enabled.
                if self.open_loop_on_no_odom:
                    # duration = distance / speed
                    vy_eff = max(1e-6, abs(self.vy))
                    self.duration = abs(self.distance_y) / vy_eff
                    self.t0 = time.monotonic()
                    self.get_logger().warn(
                        f"No /odom; using open-loop time mode for {self.duration:.2f}s to strafe {self.distance_y:.2f}m"
                    )
                else:
                    self.get_logger().warn("No /odom; cannot start without open-loop fallback enabled.")
                    self.enabled = False
        else:
            self.publish_stop()
            # Clear open-loop state
            self.t0 = None
            self.duration = None

    def capture_start(self):
        if self.odom is None:
            self.get_logger().warn("No odom yet. Cannot start.")
            self.enabled = False
            return
        p = self.odom.pose.pose.position
        q = self.odom.pose.pose.orientation
        self.x0, self.y0 = p.x, p.y
        self.yaw0 = yaw_from_quat(q)
        self.get_logger().info(f"Start captured: x0={self.x0:.3f}, y0={self.y0:.3f}, yaw0={math.degrees(self.yaw0):.1f}deg")

    def publish_stop(self):
        self.pub.publish(Twist())

    def on_timer(self):
        if not self.enabled:
            return
        # Open-loop time-based mode (no odom)
        if self.t0 is not None and self.duration is not None:
            elapsed = time.monotonic() - self.t0
            if elapsed >= self.duration:
                self.publish_stop()
                self.enabled = False
                self.get_logger().info("Done (open-loop time strafe).")
                # Clear open-loop state
                self.t0 = None
                self.duration = None
                return

            cmd = Twist()
            cmd.linear.y = self.vy if self.distance_y > 0 else -self.vy
            # Without odometry orientation, we do not apply yaw correction
            cmd.angular.z = 0.0
            self.pub.publish(cmd)
            return

        if self.odom is None or self.x0 is None:
            return

        p = self.odom.pose.pose.position
        q = self.odom.pose.pose.orientation
        x, y = p.x, p.y
        yaw = yaw_from_quat(q)

        # displacement in odom y direction (world lateral)
        dy = y - self.y0
        err = self.distance_y - dy

        # yaw hold
        yaw_err = wrap_pi(self.yaw0 - yaw)
        wz = clamp(self.k_yaw * yaw_err, -self.wz_max, self.wz_max)

        cmd = Twist()
        if abs(err) < self.pos_tol:
            self.publish_stop()
            self.enabled = False
            self.get_logger().info("Done (holonomic strafe).")
            return

        # move +y if distance_y positive, else -y
        cmd.linear.y = self.vy if self.distance_y > 0 else -self.vy
        cmd.angular.z = wz
        self.pub.publish(cmd)

def main():
    rclpy.init()
    node = HolonomicStrafe1m()
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
