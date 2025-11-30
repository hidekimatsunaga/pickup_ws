import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PointStamped
from std_msgs.msg import Float32, Bool
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

        # === パブリッシャ ===
        self.cmd_pub = self.create_publisher(Twist, '/chaser/cmd_vel', 10)
        self.camera_swing_pub = self.create_publisher(Float32, '/cameraswingmotor/target_angle', 10)
        self.completion_pub = self.create_publisher(Bool, '/chaser/approach_completed', 10)

        # === サブスクライバ ===
        self.point_sub = self.create_subscription(
            PointStamped, '/detected_depth_points', self.point_callback, 10)
        self.camera_angle_sub = self.create_subscription(
            Float32, '/cameraswingmotor/angle', self.camera_angle_callback, 10)

        # === 状態保持用の変数 ===
        self.current_camera_angle_deg = None  # 現在のカメラ角度（度数法）

        # === 制御パラメータ (ロボット移動) ===
        self.target_frame = 'base_link'
        self.target_distance = 0.5      # ロボットと物体の目標距離 [m]
        self.stop_threshold = 0.05      # 停止判定の許容誤差 [m]
        self.kp_linear = 0.6            # 距離に対する比例ゲイン
        self.kp_angular = 0.1           # 角度に対する比例ゲイン
        self.max_linear_speed = 0.1     # 最大並進速度 [m/s]
        self.max_angular_speed = 0.05    # 最大旋回速度 [rad/s]
        
        # === 制御パラメータ (カメラの「距離に応じた」下向き制御) ===
        # 例：
        #   d >= 2.0 m  → 30度
        #   d <= 0.5 m  → 60度（かなり下向き）
        # この間は線形で変化
        self.far_distance = 2.0         # これより遠いときの距離 [m]
        self.near_distance = 0.5        # これより近いときの距離 [m]（だいたい target_distance と一致させる）
        self.far_camera_angle_deg = 30.0  # 遠いときのカメラ角度 [deg]
        self.near_camera_angle_deg = 60.0 # 近いときのカメラ角度 [deg]

        # 物理的な可動範囲（安全のためのクランプ）
        self.min_camera_angle_deg = 17.6  # 下限
        self.max_camera_angle_deg = 63.9  # 上限

        # タイムアウト処理用
        self.last_detection_time = self.get_clock().now()
        self.timer = self.create_timer(0.1, self.check_timeout)
        self.get_logger().info("Object Chaser Node has been started.")

        self.completion_notified = False

    # =========================================================
    #  コールバック
    # =========================================================

    def camera_angle_callback(self, msg: Float32):
        """現在のカメラ角度を常に更新するコールバック"""
        self.current_camera_angle_deg = msg.data

    def point_callback(self, msg: PointStamped):
        """物体を検出したときに呼ばれるメインのコールバック"""
        self.last_detection_time = self.get_clock().now()

        # ↓↓↓ ここでは camera_frame の y,z はもう使わない
        # msg は camera_color_optical_frame での 3D 位置
        # ロボット制御 & カメラ制御ともに base_link 座標に変換して距離 d を使う

        try:
            transform = self.tf_buffer.lookup_transform(
                self.target_frame, msg.header.frame_id, rclpy.time.Time())
            point_in_base_link = do_transform_point(msg, transform)
        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            self.get_logger().error(f'Could not transform point: {e}')
            return

        # base_link 座標系
        target_x = point_in_base_link.point.x
        target_y = point_in_base_link.point.y
        distance = math.sqrt(target_x**2 + target_y**2)

        # --- 処理1: ロボット本体の移動制御 ---
        self.execute_robot_control(target_x, target_y, distance)

        # --- 処理2: 距離に応じてカメラを徐々に下向きにする ---
        self.control_camera_swing(distance)

    # =========================================================
    #  カメラ制御（距離 d ベース）
    # =========================================================
    def control_camera_swing(self, distance: float):
        """
        ロボット〜物体の距離[m] から、カメラの目標角度[deg]を決めてパブリッシュする。
        近づくほど徐々に下向きにし、最終的には near_camera_angle_deg まで下げる。
        """
        if self.current_camera_angle_deg is None:
            self.get_logger().warn("Current camera angle not received yet.", throttle_duration_sec=5.0)
            return

        d_far = self.far_distance
        d_near = self.near_distance

        if distance >= d_far:
            target_deg = self.far_camera_angle_deg
        elif distance <= d_near:
            target_deg = self.near_camera_angle_deg
        else:
            # 線形補間：
            # d_far → d_near に近づくにつれて angle が far_angle → near_angle に変化
            ratio = (distance - d_near) / (d_far - d_near)  # d=d_far → 1, d=d_near → 0
            target_deg = self.near_camera_angle_deg + (self.far_camera_angle_deg - self.near_camera_angle_deg) * ratio

        # 物理的な可動範囲でクランプ
        target_deg = max(self.min_camera_angle_deg, min(self.max_camera_angle_deg, target_deg))

        # 実際にパブリッシュ
        cmd_msg = Float32()
        cmd_msg.data = target_deg
        self.camera_swing_pub.publish(cmd_msg)

        # デバッグ（うるさかったらコメントアウト）
        self.get_logger().info(f"[Camera] distance={distance:.2f} m → target_angle={target_deg:.2f} deg")

    # =========================================================
    #  ロボット制御
    # =========================================================
    def execute_robot_control(self, target_x, target_y, distance=None):
        """ロボット基準のX,Y座標からcmd_velを計算してパブリッシュする"""
        if distance is None:
            distance = math.sqrt(target_x**2 + target_y**2)

        self.get_logger().info(f"物体までの計算上の距離: {distance:.2f} m")

        # 目標： base_link から見たとき (x, y) = (target_distance, 0) にしたい
        err_x = target_x - self.target_distance   # 前後方向の誤差
        err_y = target_y - 0.0                    # 横方向の誤差

        # 距離ベースの到達判定（今まで通り）
        distance_error = distance - self.target_distance

        cmd = Twist()

        # 目標距離に到達したかどうか
        if abs(distance_error) < self.stop_threshold:
            self.stop_robot()
            self.get_logger().info("Target distance reached.")

            if not self.completion_notified:
                completion_msg = Bool()
                completion_msg.data = True
                self.completion_pub.publish(completion_msg)
                self.completion_notified = True
            return

        self.completion_notified = False

        ## 並進速度制御
        vx = self.kp_linear * err_x
        vy = self.kp_linear * err_y

        vx = max(-self.max_linear_speed, min(self.max_linear_speed, vx))
        vy = max(-self.max_linear_speed, min(self.max_linear_speed, vy))
        cmd.linear.x = vx
        cmd.linear.y = vy

        ## 旋回速度制御
        cmd.angular.z = 0.0
        self.cmd_pub.publish(cmd)

    # =========================================================
    #  タイムアウト & 停止処理
    # =========================================================
    def check_timeout(self):
        """一定時間、物体を検出できなかったらロボットを停止させる"""
        if self.get_clock().now() - self.last_detection_time > rclpy.duration.Duration(seconds=1.0):
            self.stop_robot()

    def stop_robot(self):
        """ロボットを停止させる"""
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.linear.y = 0.0
        cmd.angular.z = 0.0
        self.cmd_pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = ObjectChaserNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
