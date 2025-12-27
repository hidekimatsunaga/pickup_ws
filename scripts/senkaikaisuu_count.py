import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import math


class RotationCounter(Node):
    def __init__(self):
        super().__init__('rotation_counter')

        self.sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )

        self.last_time = None
        self.total_angle = 0.0  # [rad]

        # 方向別累積角度
        self.ccw_angle = 0.0
        self.cw_angle = 0.0

        # 符号切り替わり検出用
        self.prev_sign = 0     # -1: CW, +1: CCW, 0: none
        self.switch_count = 0

        # ノイズ対策
        self.omega_eps = 0.05  # rad/s

        self.get_logger().info('Rotation counter started.')

    def cmd_vel_callback(self, msg: Twist):
        now = self.get_clock().now().nanoseconds * 1e-9

        if self.last_time is None:
            self.last_time = now
            return

        dt = now - self.last_time
        self.last_time = now

        omega = msg.angular.z

        # デッドゾーン
        if abs(omega) < self.omega_eps:
            current_sign = 0
        else:
            current_sign = 1 if omega > 0 else -1

        # 符号切り替わり検出
        if self.prev_sign != 0 and current_sign != 0:
            if current_sign != self.prev_sign:
                self.switch_count += 1

        if current_sign != 0:
            self.prev_sign = current_sign

        # 角度積分
        self.total_angle += omega * dt

        if omega > self.omega_eps:
            self.ccw_angle += omega * dt
        elif omega < -self.omega_eps:
            self.cw_angle += abs(omega * dt)

        # 回数換算
        total_rot = abs(self.total_angle) / (2 * math.pi)
        ccw_rot = self.ccw_angle / (2 * math.pi)
        cw_rot = self.cw_angle / (2 * math.pi)

        self.get_logger().info(
            f'Total rot: {total_rot:.2f} | '
            f'CCW: {ccw_rot:.2f}, CW: {cw_rot:.2f} | '
            f'Switches: {self.switch_count}'
        )


def main():
    rclpy.init()
    node = RotationCounter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
