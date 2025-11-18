#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Int8MultiArray


class TendonCalibrationNode(Node):
    """
    SDM（スイッチ）を使って
      1) たるみ状態（switch=0）までほどく
      2) 少し余分にほどいて「確実にたるませる」
      3) 再び巻き取って switch=1 になった位置（＋少し余分）をキャリブ角として記録
    を各モータについて順番に実行するノード。
    """

    def __init__(self):
        super().__init__('tendon_calibration_node')

        # ----- パラメータ -----
        self.num_motors = self.declare_parameter('num_motors', 9).value
        # たるませる方向（度）: デフォルトでは「負の方向」にほどく前提
        self.release_step = self.declare_parameter('release_step', 5.0).value
        # たるみ除去（巻き取り）方向のステップ
        self.tension_step = self.declare_parameter('tension_step', 1.0).value
        # たるみ検出後にさらにほどく回数
        self.extra_slack_steps = self.declare_parameter('extra_slack_steps', 3).value
        # スイッチON検出後にさらに巻き取る回数
        self.extra_tension_steps = self.declare_parameter('extra_tension_steps', 1).value
        # 安全用の角度範囲
        self.min_angle = self.declare_parameter('min_angle', -360.0).value
        self.max_angle = self.declare_parameter('max_angle',  360.0).value

        # ----- Pub / Sub -----
        # 司令角度を流す → AngleSerialNode がシリアルに流してくれる
        self.cmd_pub = self.create_publisher(
            Float32MultiArray,
            '/motor_angles',
            10
        )

        # 現在角度 / スイッチ状態を取得
        self.current_angles = None  # type: list[float] | None
        self.current_switch = None  # type: list[int]   | None

        self.angle_sub = self.create_subscription(
            Float32MultiArray,
            '/motor_current_angles',
            self.angle_callback,
            10
        )

        self.switch_sub = self.create_subscription(
            Int8MultiArray,
            '/switch',
            self.switch_callback,
            10
        )

        # ----- 状態管理 -----
        self.cmd_angles = [0.0] * self.num_motors
        self.calib_angles = [None] * self.num_motors

        self.current_motor = 0
        self.state = 'WAIT_DATA'  # WAIT_DATA, SLACK, SLACK_MARGIN, TENSION, TENSION_MARGIN, DONE
        self.slack_margin_count = 0
        self.tension_margin_count = 0

        # 制御ループ
        self.timer = self.create_timer(0.1, self.control_loop)

        self.get_logger().info('TendonCalibrationNode started.')
        self.get_logger().info('Waiting for /motor_current_angles and /switch ...')

    # ----- コールバック -----
    def angle_callback(self, msg: Float32MultiArray):
        if len(msg.data) != self.num_motors:
            self.get_logger().warn(f'Received angle length {len(msg.data)} != {self.num_motors}')
            return
        self.current_angles = list(msg.data)

    def switch_callback(self, msg: Int8MultiArray):
        if len(msg.data) != self.num_motors:
            self.get_logger().warn(f'Received switch length {len(msg.data)} != {self.num_motors}')
            return
        self.current_switch = list(msg.data)

    # ----- メイン制御ループ -----
    def control_loop(self):
        # まずセンサ値がそろうのを待つ
        if self.state == 'WAIT_DATA':
            if self.current_angles is None or self.current_switch is None:
                return
            # 現在角度をそのまま初期司令にする
            self.cmd_angles = list(self.current_angles)
            self.publish_cmd()
            self.state = 'SLACK'
            self.get_logger().info(
                f'Start calibration. num_motors={self.num_motors}, '
                f'release_step={self.release_step}, tension_step={self.tension_step}'
            )
            self.get_logger().info(f'Starting from current angles: {self.current_angles}')
            return

        if self.state == 'DONE':
            # 最後に一度だけ結果を表示して終了指示
            return

        # センサが来ていないときは何もしない
        if self.current_angles is None or self.current_switch is None:
            return

        i = self.current_motor
        sw_i = self.current_switch[i]
        angle_i = self.cmd_angles[i]

        # 安全チェック
        if angle_i < self.min_angle or angle_i > self.max_angle:
            self.get_logger().error(
                f'Motor {i} angle {angle_i:.2f} out of bounds '
                f'[{self.min_angle}, {self.max_angle}]. Aborting this motor.'
            )
            self.calib_angles[i] = None
            self._next_motor_or_finish()
            return

        if self.state == 'SLACK':
            # switch=1 → まだ張っているので release_step だけほどく
            # switch=0 → たるみ検出
            if sw_i == 0:
                self.get_logger().info(f'Motor {i}: slack detected (switch=0). Extra slack steps...')
                self.slack_margin_count = 0
                self.state = 'SLACK_MARGIN'
            else:
                # たるませる方向（ここではマイナス方向）に回す
                self.cmd_angles[i] = angle_i - self.release_step
                self.publish_cmd()

        elif self.state == 'SLACK_MARGIN':
            if self.slack_margin_count < self.extra_slack_steps:
                self.cmd_angles[i] = angle_i - self.release_step
                self.slack_margin_count += 1
                self.publish_cmd()
            else:
                self.get_logger().info(f'Motor {i}: slack margin done. Now tensioning...')
                self.state = 'TENSION'

        elif self.state == 'TENSION':
            # switch=0 → まだたるんでいるので巻き取る
            # switch=1 → 張り検出
            if sw_i == 1:
                self.get_logger().info(f'Motor {i}: switch ON (tension detected). Extra tension steps...')
                self.tension_margin_count = 0
                self.state = 'TENSION_MARGIN'
            else:
                self.cmd_angles[i] = angle_i + self.tension_step
                self.publish_cmd()

        elif self.state == 'TENSION_MARGIN':
            if self.tension_margin_count < self.extra_tension_steps:
                self.cmd_angles[i] = angle_i + self.tension_step
                self.tension_margin_count += 1
                self.publish_cmd()
            else:
                # キャリブレーション完了。現在角度を記録。
                # （センサ角度があればそれを優先）
                calib_angle = (
                    self.current_angles[i] if self.current_angles is not None else self.cmd_angles[i]
                )
                self.calib_angles[i] = calib_angle
                self.get_logger().info(f'Motor {i}: calibration angle = {calib_angle:.2f} [deg]')
                self._next_motor_or_finish()

    def _next_motor_or_finish(self):
        if self.current_motor < self.num_motors - 1:
            self.current_motor += 1
            self.slack_margin_count = 0
            self.tension_margin_count = 0
            self.state = 'SLACK'
            self.get_logger().info(f'--- Move to next motor: {self.current_motor} ---')
        else:
            self.state = 'DONE'
            self.get_logger().info('=== Calibration finished ===')
            self.get_logger().info(f'Calibrated angles (deg): {self.calib_angles}')
            # YAML や hpp に貼りやすい形で出す
            arr_str = ', '.join(f'{a:.2f}' if a is not None else 'null'
                                for a in self.calib_angles)
            self.get_logger().info(f'MOTOR_INIT_POS = [{arr_str}]')

    def publish_cmd(self):
        msg = Float32MultiArray()
        msg.data = list(self.cmd_angles)
        self.cmd_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = TendonCalibrationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
