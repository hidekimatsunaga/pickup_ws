import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PointStamped
from std_msgs.msg import Float32, Bool
import math


class SimpleObjectChaserNode(Node):
    """カメラ座標系のみを使って左右を判定し、yを固定オフセットで調整して移動指令を出すシンプルなノード

    前提:
    - 検出は `/detected_depth_points` (PointStamped) トピックから来る。
      PointStamped.point はカメラ座標系で、 z が奥行き（前方向）、 y が左右成分として扱う。
    - TF は使用せず、カメラ座標系の z,y を使っておおよその移動指令を作る。
    パラメータ:
    - y_offset: 物体が右/左のどちらかにいるときに用いる目標横オフセット [m]
    - target_distance: 前方の目標距離 [m]
    """

    def __init__(self):
        super().__init__('object_chaser_simple_camera_node')

        # === パブリッシャ ===
        self.cmd_pub = self.create_publisher(Twist, '/chaser/cmd_vel', 10)
        self.completion_pub = self.create_publisher(Bool, '/chaser/approach_completed', 10)

        # === サブスクライバ ===
        self.point_sub = self.create_subscription(
            PointStamped, '/detected_depth_points', self.point_callback, 10)

        # === パラメータ ===
        self.declare_parameter('y_offset', 0.18)  # 右/左に移動させたい横オフセット [m]
        self.declare_parameter('target_distance', 0.5)
        self.declare_parameter('kp_linear', 0.6)
        self.declare_parameter('kp_angular', 0.1)
        self.declare_parameter('max_linear_speed', 0.1)
        self.declare_parameter('max_angular_speed', 0.1)
        self.declare_parameter('stop_threshold', 0.05)

        self.y_offset = self.get_parameter('y_offset').value
        self.target_distance = self.get_parameter('target_distance').value
        self.kp_linear = self.get_parameter('kp_linear').value
        self.kp_angular = self.get_parameter('kp_angular').value
        self.max_linear_speed = self.get_parameter('max_linear_speed').value
        self.max_angular_speed = self.get_parameter('max_angular_speed').value
        self.stop_threshold = self.get_parameter('stop_threshold').value

        # タイムアウト処理
        self.last_detection_time = self.get_clock().now()
        self.timer = self.create_timer(0.1, self.check_timeout)

        self.completion_notified = False

        self.get_logger().info('Simple Object Chaser Node started (camera-only).')

    def point_callback(self, msg: PointStamped):
        """カメラ座標系の点を受け取り、左右判定して移動指令を算出する"""
        self.last_detection_time = self.get_clock().now()

        # カメラ座標系での点
        cam_y = msg.point.y
        cam_z = msg.point.z

        # Zが不正（近すぎる/ゼロなど）は無視
        if cam_z is None or cam_z <= 0.01:
            self.get_logger().warn('Received invalid depth (z).')
            return

        # 左右判定: cam_y > 0 -> 左、cam_y < 0 -> 右（座標系の定義に合わせて変更して下さい）
        side_sign = 1.0 if cam_y > 0.0 else -1.0

        # ここが差分: 実際の y ではなく、左右に固定のオフセットを目標 y とする
        target_x = cam_z  # カメラ座標系の z を前方距離として利用
        target_y = side_sign * abs(self.y_offset)

        # ロボット移動制御を計算
        self.execute_robot_control(target_x, target_y)

    def execute_robot_control(self, target_x, target_y):
        """前方距離 target_x と横オフセット target_y から cmd_vel を算出してパブリッシュ"""
        distance = math.sqrt(target_x**2 + target_y**2)
        angle_to_target = math.atan2(target_y, target_x)
        distance_error = distance - self.target_distance

        cmd = Twist()

        if abs(distance_error) < self.stop_threshold:
            self.stop_robot()
            if not self.completion_notified:
                completion_msg = Bool()
                completion_msg.data = True
                self.completion_pub.publish(completion_msg)
                self.completion_notified = True
            return

        self.completion_notified = False

        # 並進速度
        speed = self.kp_linear * distance_error
        speed = max(-self.max_linear_speed, min(self.max_linear_speed, speed))
        cmd.linear.x = speed * math.cos(angle_to_target)
        cmd.linear.y = speed * math.sin(angle_to_target)

        # 旋回速度
        cmd.angular.z = self.kp_angular * angle_to_target
        cmd.angular.z = max(-self.max_angular_speed, min(self.max_angular_speed, cmd.angular.z))

        self.cmd_pub.publish(cmd)

    def check_timeout(self):
        if self.get_clock().now() - self.last_detection_time > rclpy.duration.Duration(seconds=1.0):
            self.stop_robot()

    def stop_robot(self):
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.linear.y = 0.0
        cmd.angular.z = 0.0
        self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = SimpleObjectChaserNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
