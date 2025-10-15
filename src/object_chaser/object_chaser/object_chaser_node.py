import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PointStamped
from std_msgs.msg import Float32
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

        # === サブスクライバ ===
        self.point_sub = self.create_subscription(
            PointStamped, '/detected_depth_points', self.point_callback, 10)
        self.camera_angle_sub = self.create_subscription(
            Float32, '/cameraswingmotor/angle', self.camera_angle_callback, 10)

        # === 状態保持用の変数 ===
        self.current_camera_angle_deg = None # 現在のカメラ角度（度数法）

        # === 制御パラメータ (ロボット移動) ===
        self.target_frame = 'base_link'
        self.target_distance = 0.5      # ロボットと物体の目標距離 [m]
        self.stop_threshold = 0.05      # 停止判定の許容誤差 [m]
        self.kp_linear = 0.6            # 距離に対する比例ゲイン
        self.kp_angular = 0.1           # 角度に対する比例ゲイン
        self.max_linear_speed = 0.3     # 最大並進速度 [m/s]
        self.max_angular_speed = 0.8    # 最大旋回速度 [rad/s]
        
        # === 制御パラメータ (カメラ追従) ===
        self.kp_camera_swing = -1.0     # 角度(rad)ベースの比例ゲイン (要調整！)
        self.min_camera_angle_deg = 17.6 # カメラの物理的な可動範囲の下限 (度)
        self.max_camera_angle_deg = 63.9 # カメラの物理的な可動範囲の上限 (度)

        # タイムアウト処理用
        self.last_detection_time = self.get_clock().now()
        self.timer = self.create_timer(0.1, self.check_timeout)
        self.get_logger().info("Object Chaser Node has been started.")

    def camera_angle_callback(self, msg: Float32):
        """現在のカメラ角度を常に更新するコールバック"""
        self.current_camera_angle_deg = msg.data

    def point_callback(self, msg: PointStamped):
        """物体を検出したときに呼ばれるメインのコールバック"""
        self.last_detection_time = self.get_clock().now()

        # --- 処理1: カメラの追従制御 ---
        # msgはカメラ座標系での物体の3次元位置
        camera_frame_y = msg.point.y
        camera_frame_z = msg.point.z
        self.control_camera_swing(camera_frame_y, camera_frame_z)

        # --- 処理2: ロボット本体の移動制御 ---
        try:
            transform = self.tf_buffer.lookup_transform(
                self.target_frame, msg.header.frame_id, rclpy.time.Time())
            point_in_base_link = do_transform_point(msg, transform)
            self.execute_robot_control(point_in_base_link.point.x, point_in_base_link.point.y)
        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            self.get_logger().error(f'Could not transform point for robot control: {e}')
            return

    def control_camera_swing(self, y_pos, z_pos):
        """物体の3次元Y,Z座標からカメラの目標角度を計算してパブリッシュする"""
        if self.current_camera_angle_deg is None:
            self.get_logger().warn("Current camera angle not received yet.", throttle_duration_sec=5)
            return
            
        # Zが0に近い(近すぎる)場合は計算しない
        if abs(z_pos) < 0.1:
            return

        # YとZから、カメラ正面からの垂直方向の角度ズレを計算 (結果はラジアン)
        angle_error_rad = math.atan2(y_pos, z_pos)
        
        # 現在の角度をラジアンに変換
        current_camera_angle_rad = math.radians(self.current_camera_angle_deg)
        
        # 新しい目標角度を計算 (現在の角度 - 補正量)
        # kpがマイナスなので、ズレ(angle_error_rad)を引くことで追従する
        new_target_angle_rad = current_camera_angle_rad - (self.kp_camera_swing * angle_error_rad)

        # ラジアンを度数法に戻す
        new_target_angle_deg = math.degrees(new_target_angle_rad)

        # 物理的な可動範囲内に目標値を制限 (クランプ)
        new_target_angle_deg = max(self.min_camera_angle_deg, min(self.max_camera_angle_deg, new_target_angle_deg))

        self.get_logger().info(f"カメラへの指令角度: {new_target_angle_deg:.2f} 度")

        # 新しい目標角度をパブリッシュ
        cmd_msg = Float32()
        cmd_msg.data = new_target_angle_deg
        self.camera_swing_pub.publish(cmd_msg)

    def execute_robot_control(self, target_x, target_y):
        """ロボット基準のX,Y座標からcmd_velを計算してパブリッシュする"""
        distance = math.sqrt(target_x**2 + target_y**2)
        self.get_logger().info(f"物体までの計算上の距離: {distance:.2f} m")

        angle_to_target = math.atan2(target_y, target_x)
        distance_error = distance - self.target_distance

        cmd = Twist()

        if abs(distance_error) < self.stop_threshold:
            self.stop_robot()
            self.get_logger().info("Target distance reached.")
            return

        # 並進速度の計算 (同時制御)
        speed = self.kp_linear * distance_error
        speed = max(-self.max_linear_speed, min(self.max_linear_speed, speed))
        cmd.linear.x = speed * math.cos(angle_to_target)
        cmd.linear.y = speed * math.sin(angle_to_target)

        # 旋回速度の計算
        cmd.angular.z = self.kp_angular * angle_to_target
        cmd.angular.z = max(-self.max_angular_speed, min(self.max_angular_speed, cmd.angular.z))

        self.cmd_pub.publish(cmd)

    def check_timeout(self):
        """一定時間、物体を検出できなかったらロボットを停止させる"""
        if self.get_clock().now() - self.last_detection_time > rclpy.duration.Duration(seconds=1.0):
            # self.get_logger().info("Detection timed out. Stopping robot.") # ログが多すぎる場合はコメントアウト
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