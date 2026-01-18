#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool

class DiffTurnGoTurnLateral1m(Node):
    """
    Lateral move approximation for diff-drive:
      1) Turn +90 deg
      2) Go forward 1.0 m
      3) Turn -90 deg back to original yaw

        NOTE:
            This version does NOT use /odom. It runs open-loop by commanding
            constant velocity for computed durations (distance/v, angle/wz).
            You may need to tune 'v' and 'turn_wz' to match your robot.
    """

    def __init__(self):
        super().__init__('diff_turn_go_turn_lateral_1m')

        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('enable_topic', '/move_test/enable')

        self.declare_parameter('distance', 1.0)   # [m]
        self.declare_parameter('v', 0.25)         # [m/s]

        self.declare_parameter('turn_angle_deg', 90.0)  # [deg]
        self.declare_parameter('turn_wz', 0.8)          # [rad/s]
        self.declare_parameter('turn_sign', 1.0)        # +1: left, -1: right
        self.declare_parameter('settle_time', 0.15)     # [s] stop between stages
        self.declare_parameter('max_total_time', 30.0)  # [s] safety timeout

        self.distance = float(self.get_parameter('distance').value)
        self.v = float(self.get_parameter('v').value)

        self.turn_angle_rad = math.radians(float(self.get_parameter('turn_angle_deg').value))
        self.turn_wz = float(self.get_parameter('turn_wz').value)
        self.turn_sign = float(self.get_parameter('turn_sign').value)
        self.settle_time = float(self.get_parameter('settle_time').value)
        self.max_total_time = float(self.get_parameter('max_total_time').value)

        self.enabled = False

        self.state = "IDLE"
        self.stage_t0_s = None
        self.total_t0_s = None

        self.turn1_time_s = None
        self.go_time_s = None
        self.turn2_time_s = None

        self.wz1 = 0.0
        self.wz2 = 0.0
        self.vx = 0.0

        self.sub_en = self.create_subscription(Bool, self.get_parameter('enable_topic').value, self.cb_enable, 10)
        self.pub = self.create_publisher(Twist, self.get_parameter('cmd_vel_topic').value, 10)

        self.timer = self.create_timer(0.02, self.on_timer)  # 50 Hz
        self.get_logger().info("DiffTurnGoTurn(open-loop) ready. Publish /move_test/enable true to start.")

    def cb_enable(self, msg: Bool):
        self.enabled = bool(msg.data)
        if self.enabled:
            self.start_sequence()
        else:
            self.publish_stop()
            self.state = "IDLE"

    def now_s(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def start_stage(self, state: str):
        self.state = state
        self.stage_t0_s = self.now_s()

    def start_sequence(self):
        # Compute durations (open-loop)
        if abs(self.v) < 1e-6:
            self.get_logger().error("Parameter 'v' is too small (close to 0).")
            self.enabled = False
            self.state = "IDLE"
            return
        if abs(self.turn_wz) < 1e-6:
            self.get_logger().error("Parameter 'turn_wz' is too small (close to 0).")
            self.enabled = False
            self.state = "IDLE"
            return
        sign = 1.0 if self.turn_sign >= 0.0 else -1.0
        angle = self.turn_angle_rad * sign

        self.turn1_time_s = abs(angle) / abs(self.turn_wz)
        self.turn2_time_s = self.turn1_time_s
        self.go_time_s = abs(self.distance) / abs(self.v)

        self.wz1 = math.copysign(abs(self.turn_wz), angle)
        self.wz2 = -self.wz1
        self.vx = math.copysign(abs(self.v), self.distance)

        self.total_t0_s = self.now_s()
        self.start_stage("TURN1")

        self.get_logger().info(
            f"Start open-loop: turn={math.degrees(angle):.1f}deg @ {self.wz1:.2f}rad/s ({self.turn1_time_s:.2f}s), "
            f"go={self.distance:.2f}m @ {self.vx:.2f}m/s ({self.go_time_s:.2f}s)"
        )

    def publish_stop(self):
        self.pub.publish(Twist())

    def on_timer(self):
        if not self.enabled:
            return
        if self.state == "IDLE":
            return

        now = self.now_s()
        if self.total_t0_s is not None and (now - self.total_t0_s) > self.max_total_time:
            self.get_logger().error("Timeout: exceeded max_total_time. Stopping.")
            self.publish_stop()
            self.enabled = False
            self.state = "IDLE"
            return

        if self.stage_t0_s is None:
            self.stage_t0_s = now
        t = now - self.stage_t0_s

        cmd = Twist()

        if self.state == "TURN1":
            if t < self.turn1_time_s:
                cmd.angular.z = self.wz1
            else:
                self.publish_stop()
                self.start_stage("PAUSE1")
                self.get_logger().info("TURN1 done -> PAUSE1")

        elif self.state == "PAUSE1":
            if t >= self.settle_time:
                self.start_stage("GO")
                self.get_logger().info("PAUSE1 done -> GO")

        elif self.state == "GO":
            if t < self.go_time_s:
                cmd.linear.x = self.vx
            else:
                self.publish_stop()
                self.start_stage("PAUSE2")
                self.get_logger().info("GO done -> PAUSE2")

        elif self.state == "PAUSE2":
            if t >= self.settle_time:
                self.start_stage("TURN2")
                self.get_logger().info("PAUSE2 done -> TURN2")

        elif self.state == "TURN2":
            if t < self.turn2_time_s:
                cmd.angular.z = self.wz2
            else:
                self.publish_stop()
                self.enabled = False
                self.state = "IDLE"
                self.get_logger().info("Done (open-loop turn-go-turn).")
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
