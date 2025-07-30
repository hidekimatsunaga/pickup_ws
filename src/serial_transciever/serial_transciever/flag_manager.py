import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Bool
from math import fabs

class FlagManager(Node):
    def __init__(self):
        super().__init__('flag_manager')

        # ★パラメータ
        self.declare_parameter('off_angle', 37.0)      # OFF させたい角度
        self.declare_parameter('tolerance',  1.0)      # 許容幅 [deg]
        self.declare_parameter('motor_index', 9)       # 監視するモータ番号 (0-based)

        self.off_angle = self.get_parameter('off_angle').value
        self.tol       = self.get_parameter('tolerance').value
        self.idx       = self.get_parameter('motor_index').value

        # 状態
        self.wait_reach = False     # OFF 角が発行されたら True
        self.flag_sent  = False     # OFF フラグを送ったか

        # Publisher
        self.flag_pub = self.create_publisher(Bool, '/suction_flag', 10)

        # Subscribers
        self.create_subscription(
            Float32MultiArray,
            '/motor_angles',
            self.target_cb,
            10)

        self.create_subscription(
            Float32MultiArray,
            '/motor_current_angles',
            self.current_cb,
            10)

        self.get_logger().info('FlagManager ready')

    # --- コールバック ---
    def target_cb(self, msg: Float32MultiArray):
        # OFF 角が出たら待機モードへ
        if len(msg.data) > self.idx and fabs(msg.data[self.idx] - self.off_angle) < 1e-6:
            self.wait_reach = True
            self.flag_sent  = False   # 再利用に備えてリセット
            self.get_logger().info('OFF angle command detected; waiting for reach')

    def current_cb(self, msg: Float32MultiArray):
        if not self.wait_reach or self.flag_sent:
            return  # 到達待ちでない／すでに送信済み

        if len(msg.data) <= self.idx:
            return  # 範囲外防止

        if fabs(msg.data[self.idx] - self.off_angle) <= self.tol:
            # 目標到達と判断 → flag OFF
            self.flag_pub.publish(Bool(data=False))
            self.flag_sent  = True
            self.wait_reach = False
            self.get_logger().info(f'Angle reached ({msg.data[self.idx]:.2f}°) → suction_flag false')

def main():
    rclpy.init()
    rclpy.spin(FlagManager())
    rclpy.shutdown()

if __name__ == '__main__':
    main()
