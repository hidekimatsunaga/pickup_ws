#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool


class FaceTargetNoVy(Node):
    """
    Open-loop version that does not rely on odom.
    - Only commands vx and wz (no vy)
    - Runs timed TURN1 -> GO -> TURN2 sequence (any stage can be disabled)
    - Defaults: +60deg turn, short forward, -90deg turn
    - Useful when odom is unavailable; uses only the parameters given at start
    """

    def __init__(self):
        super().__init__('face_target_no_vy')

        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('enable_topic', '/move_test/enable')

        # TURN1 -> GO -> TURN2 sequence (no odom)
        self.declare_parameter('open_loop_turn1_angle_deg', -55.0)  # positive=CCW
        self.declare_parameter('open_loop_turn1_wz', 0.8)          # [rad/s]; sign derived from angle
        self.declare_parameter('open_loop_turn1_time', 0.0)        # [s]; if 0, computed from angle/wz
        self.declare_parameter('open_loop_distance', 0.8)          # [m] GO distance along +x
        self.declare_parameter('open_loop_v', 0.25)                # [m/s] forward speed (signed)
        self.declare_parameter('open_loop_wz_go', 0.0)             # [rad/s] yaw during GO (for gentle arcs)
        self.declare_parameter('open_loop_turn2_angle_deg', 120.0) # negative=CW
        self.declare_parameter('open_loop_turn2_wz', 0.8)          # [rad/s]; sign derived from angle
        self.declare_parameter('open_loop_turn2_time', 0.0)        # [s]; if 0, computed from angle/wz

        # Safety
        self.declare_parameter('open_loop_timeout', 15.0)   # [s] hard stop timeout

        enable_topic = str(self.get_parameter('enable_topic').value)
        cmd_vel_topic = str(self.get_parameter('cmd_vel_topic').value)

        self.open_loop_turn1_angle_deg = float(self.get_parameter('open_loop_turn1_angle_deg').value)
        self.open_loop_turn1_wz = float(self.get_parameter('open_loop_turn1_wz').value)
        self.open_loop_turn1_time = float(self.get_parameter('open_loop_turn1_time').value)
        self.open_loop_distance = float(self.get_parameter('open_loop_distance').value)
        self.open_loop_v = float(self.get_parameter('open_loop_v').value)
        self.open_loop_wz_go = float(self.get_parameter('open_loop_wz_go').value)
        self.open_loop_turn2_angle_deg = float(self.get_parameter('open_loop_turn2_angle_deg').value)
        self.open_loop_turn2_wz = float(self.get_parameter('open_loop_turn2_wz').value)
        self.open_loop_turn2_time = float(self.get_parameter('open_loop_turn2_time').value)
        self.open_loop_timeout = float(self.get_parameter('open_loop_timeout').value)

        self.enabled = False
        self.open_loop_stage = None  # 'TURN1', 'GO', 'TURN2'
        self.open_loop_stage_t0 = None
        self.open_loop_total_t0 = None
        self.open_loop_go_time = 0.0
        self.turn1_time_cmd = 0.0
        self.turn1_wz_cmd = 0.0
        self.turn2_time_cmd = 0.0
        self.turn2_wz_cmd = 0.0

        self.sub_en = self.create_subscription(Bool, enable_topic, self.cb_enable, 10)
        self.pub = self.create_publisher(Twist, cmd_vel_topic, 10)

        self.timer = self.create_timer(0.02, self.on_timer)
        self.get_logger().info(
            f"FaceTargetNoVy (no-odom) ready. cmd_vel={cmd_vel_topic} enable={enable_topic} distance={self.open_loop_distance:.2f}m v={self.open_loop_v:.2f} wz_go={self.open_loop_wz_go:.2f}"
        )
        self.get_logger().info('Publish enable=true to start.')

    def now_s(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def cb_enable(self, msg: Bool):
        if bool(msg.data):
            # derive turn durations/signs from angles when time not provided
            self.turn1_wz_cmd = math.copysign(abs(self.open_loop_turn1_wz), self.open_loop_turn1_angle_deg)
            ang1_rad = math.radians(self.open_loop_turn1_angle_deg)
            self.turn1_time_cmd = (
                abs(self.open_loop_turn1_time)
                if self.open_loop_turn1_time > 0.0
                else (abs(ang1_rad) / max(abs(self.open_loop_turn1_wz), 1e-6)) if abs(ang1_rad) > 1e-6 else 0.0
            )

            self.turn2_wz_cmd = math.copysign(abs(self.open_loop_turn2_wz), self.open_loop_turn2_angle_deg)
            ang2_rad = math.radians(self.open_loop_turn2_angle_deg)
            self.turn2_time_cmd = (
                abs(self.open_loop_turn2_time)
                if self.open_loop_turn2_time > 0.0
                else (abs(ang2_rad) / max(abs(self.open_loop_turn2_wz), 1e-6)) if abs(ang2_rad) > 1e-6 else 0.0
            )

            # compute GO time from distance and speed
            if abs(self.open_loop_v) > 1e-6 and self.open_loop_distance > 0.0:
                self.open_loop_go_time = self.open_loop_distance / abs(self.open_loop_v)
            else:
                self.open_loop_go_time = 0.0

            nothing_to_do = (
                self.turn1_time_cmd <= 0.0
                and self.open_loop_go_time <= 0.0
                and self.turn2_time_cmd <= 0.0
            )
            if nothing_to_do:
                self.get_logger().error('No TURN1/GO/TURN2 time; not starting.')
                self.enabled = False
                return

            self.open_loop_total_t0 = self.now_s()
            # pick first active stage
            if self.turn1_time_cmd > 0.0:
                self.open_loop_stage = 'TURN1'
            elif self.open_loop_go_time > 0.0:
                self.open_loop_stage = 'GO'
            elif self.turn2_time_cmd > 0.0:
                self.open_loop_stage = 'TURN2'
            else:
                self.publish_stop()
                self.enabled = False
                self.get_logger().warn('No active stage; stopping.')
                return

            self.open_loop_stage_t0 = self.open_loop_total_t0
            self.enabled = True
            self.get_logger().info(
                f"Starting open-loop sequence: TURN1 {self.turn1_time_cmd:.2f}s (wz={self.turn1_wz_cmd:.2f}) -> GO {self.open_loop_go_time:.2f}s (v={self.open_loop_v:.2f}, wz={self.open_loop_wz_go:.2f}) -> TURN2 {self.turn2_time_cmd:.2f}s (wz={self.turn2_wz_cmd:.2f})."
            )
        else:
            self.enabled = False
            self.open_loop_stage = None
            self.publish_stop()

    def publish_stop(self):
        self.pub.publish(Twist())

    def on_timer(self):
        if not self.enabled:
            return

        now = self.now_s()
        if self.open_loop_total_t0 is not None and (now - self.open_loop_total_t0) > self.open_loop_timeout:
            self.get_logger().error('Open-loop timeout; stopping.')
            self.enabled = False
            self.open_loop_stage = None
            self.publish_stop()
            return

        if self.open_loop_stage_t0 is None:
            self.open_loop_stage_t0 = now
        t_stage = now - self.open_loop_stage_t0

        cmd = Twist()
        if self.open_loop_stage == 'TURN1':
            if t_stage < self.turn1_time_cmd:
                cmd.angular.z = self.turn1_wz_cmd
            else:
                self.publish_stop()
                if self.open_loop_go_time > 0.0:
                    self.open_loop_stage = 'GO'
                    self.open_loop_stage_t0 = now
                    self.get_logger().info('TURN1 done -> GO')
                    return
                if self.turn2_time_cmd > 0.0:
                    self.open_loop_stage = 'TURN2'
                    self.open_loop_stage_t0 = now
                    self.get_logger().info('TURN1 done -> TURN2')
                    return
                self.enabled = False
                self.open_loop_stage = None
                self.get_logger().info('Open-loop move done (TURN1 only).')
                return

        elif self.open_loop_stage == 'GO':
            if t_stage < self.open_loop_go_time:
                cmd.linear.x = self.open_loop_v
                cmd.angular.z = self.open_loop_wz_go
            else:
                self.publish_stop()
                if self.turn2_time_cmd > 0.0:
                    self.open_loop_stage = 'TURN2'
                    self.open_loop_stage_t0 = now
                    self.get_logger().info('GO done -> TURN2')
                    return
                self.enabled = False
                self.open_loop_stage = None
                self.get_logger().info('Open-loop move done (GO only).')
                return

        elif self.open_loop_stage == 'TURN2':
            if t_stage < self.turn2_time_cmd:
                cmd.angular.z = self.turn2_wz_cmd
            else:
                self.publish_stop()
                self.enabled = False
                self.open_loop_stage = None
                self.get_logger().info('Open-loop move done (TURN2 complete).')
                return
        else:
            # Unknown stage; stop defensively
            self.get_logger().error('Unknown stage; stopping.')
            self.enabled = False
            self.open_loop_stage = None
            self.publish_stop()
            return

        self.pub.publish(cmd)


def main():
    rclpy.init()
    node = FaceTargetNoVy()
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
