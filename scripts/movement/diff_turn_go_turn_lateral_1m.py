#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
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

class DiffTurnGoTurnLateral1m(Node):
    """
    Lateral move approximation for diff-drive:
      1) Turn +90 deg
      2) Go forward 1.0 m
      3) Turn -90 deg back to original yaw
    Uses /odom for yaw and position.
    """

    def __init__(self):
        super().__init__('diff_turn_go_turn_lateral_1m')

        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('enable_topic', '/move_test/enable')

        self.declare_parameter('distance', 1.0)   # [m]
        self.declare_parameter('v', 0.25)         # [m/s]
        self.declare_parameter('k_yaw', 2.5)
        self.declare_parameter('wz_max', 1.2)     # [rad/s]
        self.declare_parameter('yaw_tol_deg', 3.0)
        self.declare_parameter('pos_tol', 0.03)

        self.distance = float(self.get_parameter('distance').value)
        self.v = float(self.get_parameter('v').value)
        self.k_yaw = float(self.get_parameter('k_yaw').value)
        self.wz_max = float(self.get_parameter('wz_max').value)
        self.yaw_tol = math.radians(float(self.get_parameter('yaw_tol_deg').value))
        self.pos_tol = float(self.get_parameter('pos_tol').value)

        self.enabled = False
        self.odom = None

        self.state = "IDLE"
        self.x0 = self.y0 = None
        self.yaw0 = None
        self.turn1_target = None
        self.turn2_target = None
        self.x_start_go = self.y_start_go = None

        self.sub_odom = self.create_subscription(Odometry, self.get_parameter('odom_topic').value, self.cb_odom, 20)
        self.sub_en = self.create_subscription(Bool, self.get_parameter('enable_topic').value, self.cb_enable, 10)
        self.pub = self.create_publisher(Twist, self.get_parameter('cmd_vel_topic').value, 10)

        self.timer = self.create_timer(0.02, self.on_timer)  # 50 Hz
        self.get_logger().info("DiffTurnGoTurn ready. Publish /move_test/enable true to start.")

    def cb_odom(self, msg: Odometry):
        self.odom = msg

    def cb_enable(self, msg: Bool):
        self.enabled = bool(msg.data)
        if self.enabled:
            self.capture_start()
        else:
            self.publish_stop()
            self.state = "IDLE"

    def capture_start(self):
        if self.odom is None:
            self.get_logger().warn("No odom yet. Cannot start.")
            self.enabled = False
            return
        p = self.odom.pose.pose.position
        q = self.odom.pose.pose.orientation
        self.x0, self.y0 = p.x, p.y
        self.yaw0 = yaw_from_quat(q)

        # +90 deg turn (to move laterally in world)
        self.turn1_target = wrap_pi(self.yaw0 + math.pi/2)
        self.turn2_target = self.yaw0  # back

        self.state = "TURN1"
        self.get_logger().info(
            f"Start: x0={self.x0:.3f}, y0={self.y0:.3f}, yaw0={math.degrees(self.yaw0):.1f}deg"
        )

    def publish_stop(self):
        self.pub.publish(Twist())

    def yaw_control(self, yaw, yaw_target):
        e = wrap_pi(yaw_target - yaw)
        wz = clamp(self.k_yaw * e, -self.wz_max, self.wz_max)
        return e, wz

    def on_timer(self):
        if not self.enabled:
            return
        if self.odom is None or self.state == "IDLE":
            return

        p = self.odom.pose.pose.position
        q = self.odom.pose.pose.orientation
        x, y = p.x, p.y
        yaw = yaw_from_quat(q)

        cmd = Twist()

        if self.state == "TURN1":
            e, wz = self.yaw_control(yaw, self.turn1_target)
            cmd.angular.z = wz
            if abs(e) < self.yaw_tol:
                self.state = "GO"
                self.x_start_go, self.y_start_go = x, y
                self.get_logger().info("TURN1 done -> GO")

        elif self.state == "GO":
            # go straight distance meters
            dx = x - self.x_start_go
            dy = y - self.y_start_go
            d = math.hypot(dx, dy)
            if abs(self.distance - d) < self.pos_tol or d > abs(self.distance):
                self.state = "TURN2"
                self.get_logger().info("GO done -> TURN2")
            else:
                cmd.linear.x = self.v if self.distance > 0 else -self.v
                # keep heading during GO
                _, wz = self.yaw_control(yaw, self.turn1_target)
                cmd.angular.z = wz

        elif self.state == "TURN2":
            e, wz = self.yaw_control(yaw, self.turn2_target)
            cmd.angular.z = wz
            if abs(e) < self.yaw_tol:
                self.publish_stop()
                self.enabled = False
                self.state = "IDLE"
                self.get_logger().info("Done (turn-go-turn).")
                return

        self.pub.publish(cmd)

def main():
    rclpy.init()
    node = DiffTurnGoTurnLateral1m()
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
