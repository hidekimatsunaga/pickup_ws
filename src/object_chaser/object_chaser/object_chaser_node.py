import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PointStamped
from std_msgs.msg import Float32, Bool
import math
import tf2_ros
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
from tf2_geometry_msgs import do_transform_point

import csv
import os
import bisect


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
        self.target_distance = 0.85      # ロボットと物体の目標距離 [m]
        self.stop_threshold = 0.05      # 停止判定の許容誤差 [m]
        self.kp_linear = 0.3            # 距離に対する比例ゲイン
        self.kp_angular = 0.1           # （今回は使ってないが一応残す）
        self.max_linear_speed = 0.1     # 最大並進速度 [m/s]
        self.max_angular_speed = 0.05   # 最大旋回速度 [rad/s]

        # === 段階的アプローチ制御パラメータ（パラメータ化） ===
        # フェーズ: 0=初期直進, 1=横方向移動で整列, 2=最終直進で距離詰め
        self.approach_phase = 0

        # パラメータ宣言（デフォルト値をここで設定）
        self.declare_parameter('approach.phase1_switch_x', 1.2)
        self.declare_parameter('approach.lateral_tolerance', 0.3)
        self.declare_parameter('approach.phase1_forward_speed', 0.08)
        self.declare_parameter('approach.phase2_lateral_speed', 0.06)
        self.declare_parameter('approach.phase3_final_forward_speed', 0.08)

        # パラメータ取得
        self.phase1_switch_x = float(self.get_parameter('approach.phase1_switch_x').get_parameter_value().double_value)
        self.lateral_tolerance = float(self.get_parameter('approach.lateral_tolerance').get_parameter_value().double_value)
        self.phase1_forward_speed = float(self.get_parameter('approach.phase1_forward_speed').get_parameter_value().double_value)
        self.phase2_lateral_speed = float(self.get_parameter('approach.phase2_lateral_speed').get_parameter_value().double_value)
        self.phase3_final_forward_speed = float(self.get_parameter('approach.phase3_final_forward_speed').get_parameter_value().double_value)

        # === 制御パラメータ (カメラの「距離に応じた」下向き制御：LUTが無いときのバックアップ) ===
        self.far_distance = 2.0         # これより遠いときの距離 [m]
        self.near_distance = 0.5        # これより近いときの距離 [m]
        self.far_camera_angle_deg = 30.0  # 遠いときのカメラ角度 [deg]
        self.near_camera_angle_deg = 60.0 # 近いときのカメラ角度 [deg]

        # 物理的な可動範囲（安全のためのクランプ）
        self.min_camera_angle_deg = 17.6  # 下限
        self.max_camera_angle_deg = 63.9  # 上限

        # === LUT（(y,z) -> angle）用の配列 ===
        self.lut_y = []
        self.lut_z = []
        self.lut_angle = []

        # LUT CSVパス（パラメータ化）
        self.declare_parameter(
            'swing_lut_csv',
            '/home/matsunaga-h/pickup_ws/src/object_chaser/csv/camera_swing_calib_yz.csv'
        )
        csv_path = self.get_parameter('swing_lut_csv').get_parameter_value().string_value

        self.load_lut(csv_path)

        # タイムアウト処理用
        self.last_detection_time = self.get_clock().now()
        self.timer = self.create_timer(0.1, self.check_timeout)
        self.get_logger().info("Object Chaser Node has been started.")

        self.completion_notified = False

        # === カメラ揺れ対策用パラメータ ===
        # 目標角の平滑化係数（0:即応、1:変化しない）
        self.declare_parameter('camera.target_smooth_alpha', 0.3)
        # 1周期あたりの最大変更量 [deg]
        self.declare_parameter('camera.max_step_deg', 2.0)
        # 目標角のデッドバンド [deg]（この範囲内の差分は無視）
        self.declare_parameter('camera.deadband_deg', 0.4)

        self.target_smooth_alpha = float(self.get_parameter('camera.target_smooth_alpha').get_parameter_value().double_value)
        self.camera_max_step_deg = float(self.get_parameter('camera.max_step_deg').get_parameter_value().double_value)
        self.camera_deadband_deg = float(self.get_parameter('camera.deadband_deg').get_parameter_value().double_value)

        # 平滑化後の目標角の保持
        self._smoothed_target_deg = None

    # =========================================================
    #  LUT 読み込み
    # =========================================================
    def load_lut(self, csv_path: str):
        if not os.path.exists(csv_path):
            self.get_logger().warn(f"LUT CSV not found: {csv_path}")
            return

        try:
            ys = []
            zs = []
            angles = []
            with open(csv_path, 'r') as f:
                reader = csv.DictReader(f)
                # 期待ヘッダ: time_sec, y_cam, z_cam, yz_norm, camera_angle_deg
                for row in reader:
                    y = float(row['y_cam'])
                    z = float(row['z_cam'])
                    ang = float(row['camera_angle_deg'])
                    ys.append(y)
                    zs.append(z)
                    angles.append(ang)

            if not angles:
                self.get_logger().warn(f"LUT CSV is empty: {csv_path}")
                return

            # ソートは必須ではないが、距離順でなくても k-NN には影響しない。
            # 一応そのまま使う。
            self.lut_y = ys
            self.lut_z = zs
            self.lut_angle = angles

            self.get_logger().info(f"Loaded camera swing LUT: {csv_path}, samples={len(self.lut_angle)}")
        except Exception as e:
            self.get_logger().error(f"Failed to load LUT CSV: {csv_path}, error={e}")
            self.lut_y = []
            self.lut_z = []
            self.lut_angle = []

    # =========================================================
    #  コールバック
    # =========================================================
    def camera_angle_callback(self, msg: Float32):
        """現在のカメラ角度を常に更新するコールバック"""
        self.current_camera_angle_deg = msg.data

    def point_callback(self, msg: PointStamped):
        """物体を検出したときに呼ばれるメインのコールバック"""
        self.last_detection_time = self.get_clock().now()

        # # --- ロボット制御用：base_link に変換 ---
        # try:
        #     transform = self.tf_buffer.lookup_transform(
        #         self.target_frame, msg.header.frame_id, rclpy.time.Time())
        #     point_in_base_link = do_transform_point(msg, transform)
        # except (LookupException, ConnectivityException, ExtrapolationException) as e:
        #     self.get_logger().error(f'Could not transform point: {e}')
        #     return

        # target_x = point_in_base_link.point.x
        # target_y = point_in_base_link.point.y
        # distance = math.sqrt(target_x**2 + target_y**2)

        # # --- 処理1: ロボット本体の移動制御 ---
        # self.execute_robot_control(target_x, target_y, distance)
        # --- camera_color_optical_frame の座標をそのまま使う ---
        x_cam = msg.point.x  # 右(+)
        y_cam = msg.point.y  # 下(+)
        z_cam = msg.point.z  # 前(+)

        # ロボットの前進方向 ≒ カメラの +z
        # ロボットの左右 ≒ カメラの +x （向きが逆なら符号変える）
        target_x = z_cam      # 前後
        target_y = -x_cam      # 左右

        distance = math.sqrt(target_x**2 + target_y**2)

        # --- 処理1: ロボット本体の移動制御 ---
        self.execute_robot_control(target_x, target_y, distance)


        # --- 処理2: カメラ制御（LUT優先、無ければ距離ベースでフォールバック） ---
        y_cam = msg.point.y  # camera_color_optical_frame の y
        z_cam = msg.point.z  # camera_color_optical_frame の z
        self.control_camera_swing(y_cam, z_cam, distance)

    # =========================================================
    #  カメラ制御：LUT + フォールバック
    # =========================================================
    def control_camera_swing(self, y_cam: float, z_cam: float, distance: float):
        """
        カメラ座標系 (y_cam, z_cam) と距離を使ってカメラ角度[deg]を決める。
        1. LUTがあれば (y,z)->angle をk-NN補間
        2. LUTが無ければ distance ベースの線形制御にフォールバック
        """
        # LUTがあれば LUT を使う
        if self.lut_angle:
            target_deg = self.lookup_camera_angle_from_yz(y_cam, z_cam, k=2)
        else:
            # LUTが無い場合は、以前の距離ベースの制御をそのまま使う
            target_deg = self.control_camera_swing_by_distance(distance)

        # 物理的な可動範囲でクランプ
        target_deg = max(self.min_camera_angle_deg, min(self.max_camera_angle_deg, target_deg))

        # --- 揺れ対策: 平滑化 + レート制限 + デッドバンド ---
        prev = self._smoothed_target_deg if self._smoothed_target_deg is not None else target_deg
        # 平滑化（一次遅れ）: new = (1-alpha)*target + alpha*prev
        alpha = max(0.0, min(1.0, self.target_smooth_alpha))
        smoothed = (1.0 - alpha) * target_deg + alpha * prev

        # レート制限（1周期あたりの最大変化量）
        delta = smoothed - prev
        max_step = max(0.0, self.camera_max_step_deg)
        if delta > max_step:
            smoothed = prev + max_step
        elif delta < -max_step:
            smoothed = prev - max_step

        # デッドバンド（小さい変化は無視して直前値を維持）
        if abs(smoothed - prev) < self.camera_deadband_deg:
            smoothed = prev

        # 保持してパブリッシュ
        self._smoothed_target_deg = smoothed
        cmd_msg = Float32()
        cmd_msg.data = smoothed
        self.camera_swing_pub.publish(cmd_msg)

        # デバッグ
        self.get_logger().info(
            f"[Camera] y={y_cam:.3f}, z={z_cam:.3f}, dist={distance:.2f} -> target={target_deg:.2f} deg, cmd={smoothed:.2f} deg"
        )

    def control_camera_swing_by_distance(self, distance: float) -> float:
        """
        旧来の「距離ベース線形制御」。
        LUTが無い場合のみ呼ばれる。
        """
        d_far = self.far_distance
        d_near = self.near_distance

        if distance >= d_far:
            target_deg = self.far_camera_angle_deg
        elif distance <= d_near:
            target_deg = self.near_camera_angle_deg
        else:
            ratio = (distance - d_near) / (d_far - d_near)  # d=d_far → 1, d=d_near → 0
            target_deg = self.near_camera_angle_deg + (self.far_camera_angle_deg - self.near_camera_angle_deg) * ratio

        return target_deg

    def lookup_camera_angle_from_yz(self, y: float, z: float, k: int = 2) -> float:
        """
        LUT に基づき (y, z) からカメラ角度[deg]を推定。
        k 個の最近傍を距離の逆数重みで平均。
        """
        n = len(self.lut_angle)
        if n == 0:
            # 本来ここには来ない想定（上でチェックしている）が念のため
            return self.near_camera_angle_deg

        if k <= 0:
            k = 1
        if k > n:
            k = n

        # 距離計算
        dists = []
        for yi, zi in zip(self.lut_y, self.lut_z):
            dy = y - yi
            dz = z - zi
            d = math.hypot(dy, dz)
            dists.append(d)

        # 近い順のインデックス
        idx_sorted = sorted(range(n), key=lambda i: dists[i])

        eps = 1e-6
        num = 0.0
        den = 0.0
        for i in idx_sorted[:k]:
            d = dists[i]
            w = 1.0 / (d + eps)
            num += w * self.lut_angle[i]
            den += w

        if den <= 0.0:
            # 変なときのフォールバック：一番近いサンプル
            return self.lut_angle[idx_sorted[0]]

        return num / den

    # =========================================================
    #  ロボット制御
    # =========================================================
    def execute_robot_control(self, target_x, target_y, distance=None):
        """ロボット基準のX,Y座標からcmd_velを計算してパブリッシュする"""
        if distance is None:
            distance = math.sqrt(target_x**2 + target_y**2)

        self.get_logger().info(f"物体までの計算上の距離: {distance:.2f} m")

        # 目標： base_link から見たとき (x, y) = (target_distance, 0) にしたい
        desired_lateral_offset = 0.0
        err_x = target_x - self.target_distance   # 前後方向の誤差
        err_y = target_y - desired_lateral_offset # 横方向の誤差

        # 距離ベースの到達判定
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
            # 到達時はフェーズをリセットしておく
            self.approach_phase = 0
            return

        # まだ到達していない
        self.completion_notified = False

        # --- フェーズ制御 ---
        # フェーズ0: 初期直進 (横移動は行わない)
        if self.approach_phase == 0:
            # 前方距離が閾値以下になったら横整列フェーズへ移る
            if target_x <= self.phase1_switch_x:
                self.approach_phase = 1
                self.get_logger().info("Approach phase -> 1 (lateral alignment)")
            else:
                # 前進のみ
                vx = min(self.phase1_forward_speed, self.max_linear_speed)
                cmd.linear.x = vx
                cmd.linear.y = 0.0
                cmd.angular.z = 0.0
                self.cmd_pub.publish(cmd)
                return

        # フェーズ1: 横方向移動で整列
        if self.approach_phase == 1:
            # 横方向が十分に整列したら最終直進フェーズへ
            if abs(err_y) <= self.lateral_tolerance:
                self.approach_phase = 2
                self.get_logger().info("Approach phase -> 2 (final forward)")
            else:
                # 横方向移動（左右）: y が正なら右に、負なら左に (カメラ座標系に合わせる)
                vy = -self.phase2_lateral_speed if err_y > 0 else self.phase2_lateral_speed
                # 軽く前進成分を入れても良いが、ここでは横移動中心にする
                cmd.linear.x = 0.0
                cmd.linear.y = vy
                cmd.angular.z = 0.0
                self.cmd_pub.publish(cmd)
                return

        # フェーズ2: 最終直進で目標距離へ詰める（横方向は微修正）
        if self.approach_phase == 2:
            # 前進は比例制御、横は小さな補正だけ入れる
            vx = self.kp_linear * err_x
            vy = self.kp_linear * err_y * 0.5

            vx = max(-self.phase3_final_forward_speed, min(self.phase3_final_forward_speed, vx))
            vy = max(-self.max_linear_speed, min(self.max_linear_speed, vy))

            cmd.linear.x = vx
            cmd.linear.y = vy
            cmd.angular.z = 0.0
            self.cmd_pub.publish(cmd)
            return

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
