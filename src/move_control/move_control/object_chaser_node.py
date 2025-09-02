import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PointStamped
import math
import tf2_ros
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
from tf2_geometry_msgs import do_transform_point

class ObjectChaserNode(Node):
    def __init__(self):
        super().__init__('object_chaser_node')

        # TF2のためのバッファとリスナーを初期化
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # PublisherとSubscriberを初期化
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.point_sub = self.create_subscription(
            PointStamped,
            '/detected_depth_points',
            self.point_callback,
            10)

        # --- 制御パラメータ（ここで調整します） ---
        self.target_frame = 'base_link'     # ロボットの基準座標系
        self.target_distance = 1.0         # 目標距離 [m]
        self.stop_threshold = 0.05          # 停止判定の許容誤差 [m]
        self.kp_linear = 0.1               # 距離に対する比例ゲイン
        self.kp_angular = 0.1               # 角度に対する比例ゲイン
        self.max_linear_speed = 0.3         # 最大並進速度 [m/s]
        self.max_angular_speed = 0.8        # 最大旋回速度 [rad/s]
        
        # 制御ループ用のタイマー（検出がない時に停止させるため）
        self.last_detection_time = self.get_clock().now()
        self.timer = self.create_timer(0.1, self.check_timeout)
        self.get_logger().info("Object Chaser Node has been started.")

    def point_callback(self, msg: PointStamped):
        self.last_detection_time = self.get_clock().now()
        try:
            # camera_link座標系の点をbase_link座標系に変換
            transform = self.tf_buffer.lookup_transform(
                self.target_frame,      # 変換先のフレーム
                msg.header.frame_id,    # 変換元のフレーム
                rclpy.time.Time())      # 最新の変換を取得

            # 実際に座標変換を実行
            point_in_base_link = do_transform_point(msg, transform)
            
            target_x = point_in_base_link.point.x
            target_y = point_in_base_link.point.y

            # 制御ロジックを実行
            self.execute_control(target_x, target_y)

        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            self.get_logger().error(f'Could not transform point: {e}')
            return

    def execute_control(self, target_x, target_y):
        distance = math.sqrt(target_x**2 + target_y**2)
        
        # =======================================================
        # ▼▼▼ 変更点：距離をターミナルに表示してデバッグ ▼▼▼
        # =======================================================
        self.get_logger().info(f"物体までの計算上の距離: {distance:.2f} m")
        # =======================================================

        angle_to_target = math.atan2(target_y, target_x)
        distance_error = distance - self.target_distance

        cmd = Twist()

        if abs(distance_error) < self.stop_threshold:
            # 目標距離に到達
            self.stop_robot()
            self.get_logger().info("Target distance reached.")
            return

        # 並進速度の計算
        speed = self.kp_linear * distance_error
        speed = max(-self.max_linear_speed, min(self.max_linear_speed, speed))
        cmd.linear.x = speed * math.cos(angle_to_target)
        cmd.linear.y = speed * math.sin(angle_to_target)

        # 旋回速度の計算
        cmd.angular.z = self.kp_angular * angle_to_target
        cmd.angular.z = max(-self.max_angular_speed, min(self.max_angular_speed, cmd.angular.z))

        self.cmd_pub.publish(cmd)

    def check_timeout(self):
        # 一定時間、物体を検出できなかったら停止する
        if self.get_clock().now() - self.last_detection_time > rclpy.duration.Duration(seconds=1.0):
            self.get_logger().info("Detection timed out. Stopping robot.")
            self.stop_robot()

    def stop_robot(self):
        # ロボットを停止させる
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.linear.y = 0.0
        cmd.angular.z = 0.0
        self.cmd_pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = ObjectChaserNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()