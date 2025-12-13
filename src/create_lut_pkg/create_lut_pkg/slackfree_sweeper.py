#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Int8MultiArray


def all_tight(sw: Int8MultiArray) -> bool:
    return sw is not None and len(sw.data) > 0 and all(v == 1 for v in sw.data)


class SlackFreeSweeper(Node):
    """
    /motor_current_angles を基準に絶対角指令 /motor_angles を生成。
    指定モータ（7-9番など）を +sweep_deg まで掃引し、終端でたるみ除去ムーブを挟む。
    """

    def __init__(self):
        super().__init__('slackfree_sweeper')

        # ===== Parameters =====
        self.declare_parameter('motor_dim', 9)

        # 7〜9番モータを中心に、という要望に合わせてデフォルトは index 6,7,8
        self.declare_parameter('focus_indices', [6, 7, 8])

        # 掃引角
        self.declare_parameter('sweep_deg', 180.0)
        self.declare_parameter('step_deg', 5.0)          # 掃引時の1ステップ
        self.declare_parameter('publish_rate_hz', 20.0)  # 指令再送周期
        self.declare_parameter('hold_sec', 0.5)          # +180到達後の保持

        # たるみ判定の安定化（連続N回 tight を要求）
        self.declare_parameter('tight_required_count', 5)

        # たるみ除去ムーブ
        self.declare_parameter('slack_step_deg', 2.0)       # たるみ除去で増やす角度
        self.declare_parameter('max_slack_steps', 200)      # 無限ループ防止
        # 各モータの「巻き取り方向」(+1 or -1)
        # 例：全モータ「角度を増やすと巻き取る」なら全部 +1 のままでOK
        self.declare_parameter('wind_signs', [1, 1, 1, 1, 1, 1, 1, 1, 1])

        # トピック名（あなたの系に固定）
        self.current_topic = '/motor_current_angles'
        self.command_topic = '/motor_angles'
        self.switch_topic = '/switch'

        self.motor_dim = int(self.get_parameter('motor_dim').value)
        self.focus_indices = list(self.get_parameter('focus_indices').value)
        self.sweep_deg = float(self.get_parameter('sweep_deg').value)
        self.step_deg = float(self.get_parameter('step_deg').value)
        self.publish_rate_hz = float(self.get_parameter('publish_rate_hz').value)
        self.hold_sec = float(self.get_parameter('hold_sec').value)
        self.tight_required_count = int(self.get_parameter('tight_required_count').value)
        self.slack_step_deg = float(self.get_parameter('slack_step_deg').value)
        self.max_slack_steps = int(self.get_parameter('max_slack_steps').value)
        self.wind_signs = [int(x) for x in self.get_parameter('wind_signs').value]

        if len(self.wind_signs) != self.motor_dim:
            raise ValueError(f"wind_signs length must be {self.motor_dim}")

        for idx in self.focus_indices:
            if idx < 0 or idx >= self.motor_dim:
                raise ValueError(f"focus index out of range: {idx}")

        # ===== Pub/Sub =====
        self.pub_cmd = self.create_publisher(Float32MultiArray, self.command_topic, 10)

        self.sub_cur = self.create_subscription(
            Float32MultiArray, self.current_topic, self.cb_current, 10
        )
        self.sub_sw = self.create_subscription(
            Int8MultiArray, self.switch_topic, self.cb_switch, 10
        )

        # ===== State =====
        self.cur_angles = None  # list[float]
        self.sw = None          # Int8MultiArray

        self.tight_count = 0

        self.base_angles = None  # たるみ無しになった瞬間の基準角
        self.focus_i = 0         # focus_indices の何番目か
        self.delta = 0.0         # 現在の掃引量（0 -> sweep_deg）

        self.mode = 'WAIT_TIGHT_BASE'
        # modes:
        # WAIT_TIGHT_BASE -> SWEEPING -> HOLD_END -> SLACK_REMOVAL -> (next) WAIT_TIGHT_BASE

        self.hold_until_ns = None
        self.slack_steps_done = 0

        # 最後に送った指令（再送用）
        self.last_cmd = None

        # ===== Timer =====
        period = 1.0 / max(1e-3, self.publish_rate_hz)
        self.timer = self.create_timer(period, self.tick)

        self.get_logger().info(
            "SlackFreeSweeper started.\n"
            f"  subscribe: {self.current_topic}, {self.switch_topic}\n"
            f"  publish:   {self.command_topic}\n"
            f"  focus_indices: {self.focus_indices}\n"
            f"  sweep_deg: {self.sweep_deg}, step_deg: {self.step_deg}\n"
            f"  wind_signs: {self.wind_signs}\n"
            "  ※ wind_signs が実機と逆だと、たるみ除去が進まないので注意してください。"
        )

    def cb_current(self, msg: Float32MultiArray):
        if len(msg.data) < self.motor_dim:
            self.get_logger().warn(f"/motor_current_angles length < motor_dim: {len(msg.data)}")
            return
        self.cur_angles = list(msg.data[:self.motor_dim])

    def cb_switch(self, msg: Int8MultiArray):
        self.sw = msg
        if all_tight(msg):
            self.tight_count += 1
        else:
            self.tight_count = 0

    def publish_abs(self, angles):
        m = Float32MultiArray()
        m.data = [float(x) for x in angles]
        self.pub_cmd.publish(m)
        self.last_cmd = m

    def tick(self):
        # 再送（受け側の取りこぼし対策）
        if self.last_cmd is not None:
            self.pub_cmd.publish(self.last_cmd)

        if self.cur_angles is None or self.sw is None:
            return

        now_ns = self.get_clock().now().nanoseconds

        # たるみが出たら基本的に「進行停止」し、ベースから取り直す
        # ただし SLACK_REMOVAL 中は除去を継続
        if not all_tight(self.sw) and self.mode not in ('SLACK_REMOVAL', 'WAIT_TIGHT_BASE'):
            self.mode = 'SLACK_REMOVAL'
            self.slack_steps_done = 0

        if self.mode == 'WAIT_TIGHT_BASE':
            # たるみ無しが安定したらベース確定
            if self.tight_count >= self.tight_required_count:
                self.base_angles = self.cur_angles.copy()
                self.delta = 0.0
                self.mode = 'SWEEPING'
                self.get_logger().info(
                    f"BASE fixed. Start sweeping motor index {self.focus_indices[self.focus_i]}."
                )
            return

        if self.mode == 'SWEEPING':
            idx = self.focus_indices[self.focus_i]
            sign = self.wind_signs[idx]

            self.delta = min(self.sweep_deg, self.delta + self.step_deg)

            cmd = self.base_angles.copy()
            cmd[idx] = cmd[idx] + sign * self.delta  # ベースに対して「巻き取り方向へ+delta」

            self.publish_abs(cmd)

            if math.isclose(self.delta, self.sweep_deg, abs_tol=1e-6):
                self.mode = 'HOLD_END'
                self.hold_until_ns = now_ns + int(self.hold_sec * 1e9)
                self.get_logger().info(
                    f"Reached end (+{self.sweep_deg} deg) on motor {idx}. Hold {self.hold_sec:.2f}s."
                )
            return

        if self.mode == 'HOLD_END':
            if self.hold_until_ns is not None and now_ns >= self.hold_until_ns:
                # 終端到達後に、たるみ除去ムーブへ
                self.mode = 'SLACK_REMOVAL'
                self.slack_steps_done = 0
                self.get_logger().info("Enter slack removal phase.")
            return

        if self.mode == 'SLACK_REMOVAL':
            # たるみ無しが安定したら、次のモータへ
            if self.tight_count >= self.tight_required_count:
                # ベース更新（たるみ除去後の現在角）
                self.base_angles = self.cur_angles.copy()

                # 次のfocusへ
                self.focus_i += 1
                if self.focus_i >= len(self.focus_indices):
                    self.focus_i = 0  # ループさせる（止めたいなら DONE にする）

                self.delta = 0.0
                self.mode = 'SWEEPING'
                self.get_logger().info(
                    f"Slack removed. Update BASE. Next motor index {self.focus_indices[self.focus_i]}."
                )
                return

            # 最大回数を超えたら安全停止（これ以上巻き取っても改善しない可能性）
            if self.slack_steps_done >= self.max_slack_steps:
                self.get_logger().warn(
                    "Slack removal exceeded max_slack_steps. "
                    "Check wind_signs or mechanics. Staying in WAIT_TIGHT_BASE."
                )
                self.mode = 'WAIT_TIGHT_BASE'
                self.last_cmd = None
                return

            # たるみ除去ムーブ：全モータを巻き取り方向へ少しずつ動かす
            cmd = self.cur_angles.copy()
            for i in range(self.motor_dim):
                cmd[i] = cmd[i] + self.wind_signs[i] * self.slack_step_deg

            self.publish_abs(cmd)
            self.slack_steps_done += 1
            return


def main(args=None):
    rclpy.init(args=args)
    node = SlackFreeSweeper()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
