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
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)

def wrap_pi(a: float) -> float:
    return (a + math.pi) % (2.0 * math.pi) - math.pi

def clamp(x: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, x))

def rel_in_body_frame(x: float, y: float, yaw: float, target_xy) -> tuple[float, float]:
    dx = target_xy[0] - x
    dy = target_xy[1] - y
    c = math.cos(yaw)
    s = math.sin(yaw)
    bx = c * dx + s * dy
    by = -s * dx + c * dy
    return bx, by

class HolonomicFaceTarget(Node):
    """
    Drive a holonomic base toward a fixed target while always yawing to face it.
    - target position is given in odom frame (target_x, target_y)
    - velocity is reduced near the goal (stop_radius / slow_radius)
    - requires odom; stops automatically when the target is reached
    """

    def __init__(self):
        super().__init__('holonomic_face_target')

        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('odom_topic', '/wheel_odom')
        self.declare_parameter('enable_topic', '/move_test/enable')

        self.declare_parameter('target_x', 0.0)
        self.declare_parameter('target_y', 0.0)
        self.declare_parameter('stop_radius', 0.1)    # [m] stop when closer than this
        self.declare_parameter('slow_radius', 0.5)    # [m] begin tapering speed
        self.declare_parameter('pos_tol', 0.03)       # [m] tolerance to final standoff point
        self.declare_parameter('yaw_tol', 0.1)        # [rad] final yaw tolerance

        self.declare_parameter('v_max', 0.4)          # [m/s]
        self.declare_parameter('vy_max', 0.35)        # [m/s]
        self.declare_parameter('k_v', 1.0)            # speed gain
        self.declare_parameter('k_pos', 1.2)          # positional gain toward standoff point

        self.declare_parameter('k_yaw', 2.5)          # yaw gain
        self.declare_parameter('wz_max', 2.0)         # [rad/s]

        # Open-loop fallback when odom is unavailable
        self.declare_parameter('open_loop_on_no_odom', True)
        self.declare_parameter('open_loop_distance', 0.5)  # [m] along +x in robot frame
        self.declare_parameter('open_loop_v', 0.25)         # [m/s]
        # Open-loop goal in robot frame (at start). Robot will drive toward this point,
        # stop stop_radius handoff early, then time-turn to face the point.
        # Example: target at 45deg direction -> (0.6, 0.6) means diagonal right-forward
        self.declare_parameter('open_loop_target_x_body', 0.6)
        self.declare_parameter('open_loop_target_y_body', 0.6)
        self.declare_parameter('open_loop_stop_radius', 0.1)
        self.declare_parameter('open_loop_wz_face', 0.8)
        self.declare_parameter('open_loop_timeout', 15.0)
        self.declare_parameter('open_loop_start_yaw_rad', 0.0)
        self.declare_parameter('open_loop_blend_turn', True)
        self.declare_parameter('open_loop_wz_go_max', 10)
        self.declare_parameter('open_loop_arc_angle_deg', 60.0)        
        self.declare_parameter('open_loop_arc_direction', -1.0)  # -1.0=clockwise, 1.0=counter-clockwise
        self.target_x = float(self.get_parameter('target_x').value)
        self.target_y = float(self.get_parameter('target_y').value)
        self.stop_radius = float(self.get_parameter('stop_radius').value)
        self.slow_radius = float(self.get_parameter('slow_radius').value)
        self.pos_tol = float(self.get_parameter('pos_tol').value)
        self.yaw_tol = float(self.get_parameter('yaw_tol').value)
        self.v_max = float(self.get_parameter('v_max').value)
        self.vy_max = float(self.get_parameter('vy_max').value)
        self.k_v = float(self.get_parameter('k_v').value)
        self.k_pos = float(self.get_parameter('k_pos').value)
        self.k_yaw = float(self.get_parameter('k_yaw').value)
        self.wz_max = float(self.get_parameter('wz_max').value)
        self.open_loop_on_no_odom = bool(self.get_parameter('open_loop_on_no_odom').value)
        self.open_loop_distance = float(self.get_parameter('open_loop_distance').value)
        self.open_loop_v = float(self.get_parameter('open_loop_v').value)
        self.open_loop_target_x_body = float(self.get_parameter('open_loop_target_x_body').value)
        self.open_loop_target_y_body = float(self.get_parameter('open_loop_target_y_body').value)
        self.open_loop_stop_radius = float(self.get_parameter('open_loop_stop_radius').value)
        self.open_loop_wz_face = float(self.get_parameter('open_loop_wz_face').value)
        self.open_loop_timeout = float(self.get_parameter('open_loop_timeout').value)
        self.open_loop_start_yaw_rad = float(self.get_parameter('open_loop_start_yaw_rad').value)
        self.open_loop_blend_turn = bool(self.get_parameter('open_loop_blend_turn').value)
        self.open_loop_wz_go_max = float(self.get_parameter('open_loop_wz_go_max').value)
        self.open_loop_arc_angle_deg = float(self.get_parameter('open_loop_arc_angle_deg').value)
        self.open_loop_arc_direction = float(self.get_parameter('open_loop_arc_direction').value)

        odom_topic = str(self.get_parameter('odom_topic').value)
        enable_topic = str(self.get_parameter('enable_topic').value)
        cmd_vel_topic = str(self.get_parameter('cmd_vel_topic').value)

        self.odom = None
        self.enabled = False
        self.warned_no_odom = False
        self.open_loop_t0 = None
        self.open_loop_duration = None
        self.open_loop_sign = 1.0
        self.open_loop_stage = None  # 'GO', 'TURN'
        self.open_loop_stage_t0 = None
        self.open_loop_total_t0 = None
        self.open_loop_go_vx = 0.0
        self.open_loop_go_vy = 0.0
        self.open_loop_go_time = 0.0
        self.open_loop_go_wz = 0.0
        self.open_loop_turn_wz = 0.0
        self.open_loop_turn_time = 0.0

        self.sub_odom = self.create_subscription(Odometry, odom_topic, self.cb_odom, 30)
        self.sub_en = self.create_subscription(Bool, enable_topic, self.cb_enable, 10)
        self.pub = self.create_publisher(Twist, cmd_vel_topic, 10)

        self.timer = self.create_timer(0.02, self.on_timer)  # 50 Hz
        self.get_logger().info(
            f"HolonomicFaceTarget ready. Target=({self.target_x:.3f}, {self.target_y:.3f}) cmd_vel={cmd_vel_topic} odom={odom_topic} enable={enable_topic}"
        )
        self.get_logger().info('Publish enable=true to start.')

    def now_s(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def cb_odom(self, msg: Odometry):
        self.odom = msg
        self.warned_no_odom = False

    def cb_enable(self, msg: Bool):
        if bool(msg.data):
            if self.odom is None:
                if self.open_loop_on_no_odom:
                    if abs(self.open_loop_v) < 1e-6:
                        self.get_logger().error('open_loop_v too small; cannot start open-loop move.')
                        self.enabled = False
                        return
                    if abs(self.open_loop_wz_face) < 1e-6:
                        self.get_logger().error('open_loop_wz_face too small; cannot start open-loop turn.')
                        self.enabled = False
                        return
                    # plan staged open-loop: GO (toward target in body frame) then TURN (face target)
                    tx = self.open_loop_target_x_body
                    ty = self.open_loop_target_y_body
                    r = math.hypot(tx, ty)
                    stop = max(0.0, self.open_loop_stop_radius)
                    
                    # For arc motion facing target center:
                    # - Robot always faces target (radial direction)
                    # - Motion is tangential (perpendicular to radial = lateral in body frame)
                    # - vx=0, vy=v_tangential, wz proportional to arc
                    arc_angle = math.radians(self.open_loop_arc_angle_deg)
                    arc_length = r * arc_angle  # s = r*theta
                    
                    self.open_loop_go_time = arc_length / abs(self.open_loop_v)
                    # Tangential velocity in body frame: pure lateral (vy)
                    self.open_loop_go_vx = 0.0
                    self.open_loop_go_vy = self.open_loop_arc_direction * self.open_loop_v  # move sideways (tangential)
                    
                    # Angular velocity to complete arc_angle in go_time
                    if self.open_loop_go_time > 1e-6:
                        self.open_loop_go_wz = -self.open_loop_arc_direction * arc_angle / self.open_loop_go_time
                    else:
                        self.open_loop_go_wz = 0.0
                        self.open_loop_go_vy = 0.0
                        self.open_loop_go_time = 0.0
                        self.open_loop_go_wz = 0.0

                    # Start yaw: face target initially
                    movement_yaw = math.atan2(ty, tx) if r > 1e-6 else 0.0
                    start_yaw = movement_yaw

                    # Skip TURN stage: set turn time to 0
                    self.open_loop_turn_wz = 0.0
                    self.open_loop_turn_time = 0.0

                    # initialize state machine
                    self.open_loop_total_t0 = self.now_s()
                    if self.open_loop_go_time > 0.0:
                        self.open_loop_stage = 'GO'
                        self.open_loop_stage_t0 = self.open_loop_total_t0
                    else:
                        self.open_loop_stage = 'TURN'
                        self.open_loop_stage_t0 = self.open_loop_total_t0
                    self.enabled = True
                    self.get_logger().warn(
                        f'Odom not available; running open-loop staged move: arc_length={arc_length:.2f}m, duration={self.open_loop_go_time:.2f}s, v=(vx={self.open_loop_go_vx:.2f}, vy={self.open_loop_go_vy:.2f}), wz={self.open_loop_go_wz:.2f}.'
                    )
                    return
                self.get_logger().warn('Enable received but odom not yet available; ignoring start request.')
                self.enabled = False
                return
            self.enabled = True
            p = self.odom.pose.pose.position
            q = self.odom.pose.pose.orientation
            yaw = yaw_from_quat(q)
            self.open_loop_t0 = None
            self.open_loop_duration = None
            self.get_logger().info(
                f"Starting move toward target=({self.target_x:.3f}, {self.target_y:.3f}) from ({p.x:.3f}, {p.y:.3f}), yaw={math.degrees(yaw):.1f}deg"
            )
        else:
            self.enabled = False
            self.publish_stop()
            self.open_loop_t0 = None
            self.open_loop_duration = None
            self.open_loop_stage = None
            self.open_loop_stage_t0 = None
            self.open_loop_total_t0 = None

    def publish_stop(self):
        self.pub.publish(Twist())

    def on_timer(self):
        if not self.enabled:
            return
        # Open-loop mode when odom is unavailable
        if self.odom is None and self.open_loop_stage is not None:
            now = self.now_s()
            if self.open_loop_total_t0 is not None and (now - self.open_loop_total_t0) > self.open_loop_timeout:
                self.get_logger().error('Open-loop timeout; stopping.')
                self.publish_stop()
                self.enabled = False
                self.open_loop_stage = None
                return

            if self.open_loop_stage_t0 is None:
                self.open_loop_stage_t0 = now
            t_stage = now - self.open_loop_stage_t0

            cmd = Twist()
            if self.open_loop_stage == 'GO':
                if t_stage < self.open_loop_go_time:
                    cmd.linear.x = self.open_loop_go_vx
                    cmd.linear.y = self.open_loop_go_vy
                    cmd.angular.z = self.open_loop_go_wz
                else:
                    self.publish_stop()
                    self.open_loop_stage = 'TURN'
                    self.open_loop_stage_t0 = now
                    self.get_logger().info('Open-loop GO done -> TURN')
                    return
            elif self.open_loop_stage == 'TURN':
                if t_stage < self.open_loop_turn_time:
                    cmd.angular.z = self.open_loop_turn_wz
                else:
                    self.publish_stop()
                    self.enabled = False
                    self.open_loop_stage = None
                    self.get_logger().info('Done (open-loop staged move without odom).')
                    return
            else:
                # unknown stage; stop
                self.publish_stop()
                self.enabled = False
                self.open_loop_stage = None
                return

            self.pub.publish(cmd)
            return

        if self.odom is None:
            if not self.warned_no_odom:
                self.get_logger().warn('No odom; stopping.')
                self.warned_no_odom = True
            self.publish_stop()
            self.enabled = False
            return

        p = self.odom.pose.pose.position
        q = self.odom.pose.pose.orientation
        x, y = p.x, p.y
        yaw = yaw_from_quat(q)

        dx = self.target_x - x
        dy = self.target_y - y
        r = math.hypot(dx, dy)

        # Desired standoff pose: keep target directly in front at stop_radius
        if r < 1e-4:
            bearing = yaw  # fallback
        else:
            bearing = math.atan2(dy, dx)

        goal_x = self.target_x - self.stop_radius * math.cos(bearing)
        goal_y = self.target_y - self.stop_radius * math.sin(bearing)

        ex = goal_x - x
        ey = goal_y - y

        # transform error into body frame
        c = math.cos(yaw)
        s = math.sin(yaw)
        ex_b = c * ex + s * ey
        ey_b = -s * ex + c * ey

        # positional controller toward goal
        vx = clamp(self.k_pos * ex_b, -self.v_max, self.v_max)
        vy = clamp(self.k_pos * ey_b, -self.vy_max, self.vy_max)

        # yaw control: face the target
        yaw_err = wrap_pi(bearing - yaw)
        wz = clamp(self.k_yaw * yaw_err, -self.wz_max, self.wz_max)

        # stop condition: close to standoff point and facing target
        if math.hypot(ex, ey) <= self.pos_tol and abs(yaw_err) <= self.yaw_tol:
            self.publish_stop()
            self.enabled = False
            self.get_logger().info('Target aligned in front; stopping.')
            return

        cmd = Twist()
        cmd.linear.x = vx
        cmd.linear.y = vy
        cmd.angular.z = wz
        self.pub.publish(cmd)

def main():
    rclpy.init()
    node = HolonomicFaceTarget()
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
