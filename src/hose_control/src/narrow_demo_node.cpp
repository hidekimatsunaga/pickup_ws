#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/string.hpp>
#include <vector>
#include <string>
#include <cmath>
#include <thread>
#include <iostream>
#include <chrono>
#include <atomic>

#include "hose_control/narrow_space_controll_position.hpp"  // motor_sequences::narrow_sequence (10要素/step想定)
#include "hose_control/motor_pickup_position.hpp"           // motor_sequences::pickup_sequence (★10要素/stepにする [0..8]=1~9軸, [9]=10軸)

class AutoSequenceNode : public rclcpp::Node {
public:
  AutoSequenceNode()
  : Node("auto_sequence_node"),
    sequence_step_(-1),
    auto_mode_(false),
    auto_step_(0),
    pressure_threshold_(-110.0f),  // ★しきい値(例)。実機に合わせて調整して
    latest_pressure_(0.0f)
  {
    // --- シーケンスデータを取り込み ---
    // 手動(挿入〜位置合わせ)用
    manual_sequence_ = motor_sequences::narrow_sequence;
    // 自動回収(引き抜き/回収)用
    // pickup_sequence_[i] は {m1,...,m9,m10} の10要素にしておくこと
    pickup_sequence_ = motor_sequences::pickup_sequence;

    // --- Publisher ---
    auto qos = rclcpp::QoS(rclcpp::KeepLast(50)).reliable();
    pub_motor1_9_    = this->create_publisher<std_msgs::msg::Float32MultiArray>("/motor_angles", qos);
    pub_motor10_     = this->create_publisher<std_msgs::msg::Float32>("/chokudomotor/target_angle", 10);
    pub_robot_state_ = this->create_publisher<std_msgs::msg::String>("/robot/state", 10);

    // --- Subscriber: 気圧センサ(負圧検出) ---
    pressure_sub_ = this->create_subscription<std_msgs::msg::Float32>(
      "/sensor/pressure", 10,
      std::bind(&AutoSequenceNode::pressureCallback, this, std::placeholders::_1)
    );

    // --- タイマー: autoモード用のステップ送り ---
    using namespace std::chrono_literals;
    auto_timer_ = this->create_wall_timer(
      500ms,  // 0.5秒ごとに次のステップを送る（適宜調整）
      std::bind(&AutoSequenceNode::autoSequenceLoop, this)
    );

    RCLCPP_INFO(this->get_logger(),
      "AutoSequenceNode started.\n"
      "Manual keys: [n] next, [b] back, [q] quit\n"
      "Auto mode: pressure > %.2f triggers pickup_sequence playback.",
      pressure_threshold_
    );

    // --- キーボード入力監視スレッド (手動モード用UI) ---
    input_thread_ = std::thread([this]() { this->keyboardLoop(); });
    input_thread_.detach();
  }

private:
  //========================
  // ROS Pub/Sub/Timer
  //========================
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pub_motor1_9_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr           pub_motor10_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr            pub_robot_state_;

  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr        pressure_sub_;
  rclcpp::TimerBase::SharedPtr                                   auto_timer_;

  //========================
  // シーケンス関連データ
  //========================
  std::vector<std::vector<float>> manual_sequence_;   // narrow_sequence (10要素/step想定)
  std::vector<std::vector<float>> pickup_sequence_;   // pickup_sequence (★10要素/step想定)

  int sequence_step_;      // 手動モードの現在ステップ (n/bで進める)
  std::thread input_thread_;

  //========================
  // オートピックアップ状態
  //========================
  std::atomic<bool> auto_mode_;     // trueならpickup_sequenceを自動実行中
  size_t            auto_step_;     // pickup_sequence_ のどこまで送ったか
  float             pressure_threshold_;
  float             latest_pressure_;

  //========================
  // キーボード入力ループ
  //========================
  void keyboardLoop() {
    char input;
    while (rclcpp::ok()) {
      std::cout << "\n[n] Next  [b] Back  [q] Quit  → ";
      std::cin >> input;

      if (input == 'n') {
        // 手動で前に進む（挿入/アプローチ側のシナリオ）
        publishManualStep(+1);
      } else if (input == 'b') {
        // 手動で戻る
        publishManualStep(-1);
      } else if (input == 'q') {
        RCLCPP_INFO(this->get_logger(), "Exiting program...");
        rclcpp::shutdown();
        break;
      } else {
        std::cout << "Invalid key. Use [n], [b], or [q]." << std::endl;
      }
    }
  }

  //========================
  // 手動シーケンス送信 (narrow側)
  //========================
  void publishManualStep(int direction) {
    // autoモード中は手動コマンド無視（安全のため）
    if (auto_mode_.load()) {
      RCLCPP_WARN(this->get_logger(),
        "Ignored manual input because auto pickup mode is active.");
      return;
    }

    sequence_step_ += direction;

    if (manual_sequence_.empty()) {
      RCLCPP_ERROR(this->get_logger(), "Manual sequence data is empty!");
      return;
    }

    if (sequence_step_ < 0) {
      sequence_step_ = -1;
      RCLCPP_WARN(this->get_logger(), "At the beginning. Press 'n' to start.");
      return;
    }

    if (static_cast<size_t>(sequence_step_) >= manual_sequence_.size()) {
      sequence_step_ = manual_sequence_.size() - 1;
      RCLCPP_WARN(this->get_logger(), "Already at the last manual step.");
      return;
    }

    const auto &target_angles_all = manual_sequence_[sequence_step_];

    // --- Motor1-9 publish ---
    if (target_angles_all.size() < 9) {
      RCLCPP_ERROR(this->get_logger(),
        "Manual step %d has less than 9 values!", sequence_step_);
      return;
    }

    std_msgs::msg::Float32MultiArray motor1_9_msg;
    motor1_9_msg.data.assign(
      target_angles_all.begin(),
      target_angles_all.begin() + 9
    );
    pub_motor1_9_->publish(motor1_9_msg);

    // --- Motor10 publish (手動シーケンスでは10軸もある想定) ---
    if (target_angles_all.size() >= 10) {
      std_msgs::msg::Float32 motor10_msg;
      motor10_msg.data = target_angles_all[9];
      pub_motor10_->publish(motor10_msg);
      RCLCPP_INFO(this->get_logger(),
                  "Manual step %d/%zu → Pub motor1-9 + motor10(%.2f)",
                  sequence_step_, manual_sequence_.size() - 1, motor10_msg.data);
    } else {
      RCLCPP_INFO(this->get_logger(),
                  "Manual step %d/%zu → Pub motor1-9 (no motor10 in data)",
                  sequence_step_, manual_sequence_.size() - 1);
    }

    // --- 「collecting」トリガー ---
    if (sequence_step_ == static_cast<int>(manual_sequence_.size()) - 2) {
      std_msgs::msg::String state_msg;
      state_msg.data = "collecting";
      pub_robot_state_->publish(state_msg);

      RCLCPP_INFO(this->get_logger(),
        "Reached collecting prep step → Published /robot/state = 'collecting'");
    }
  }

  //========================
  // 気圧センサコールバック
  //========================
  void pressureCallback(const std_msgs::msg::Float32::SharedPtr msg)
  {
    latest_pressure_ = msg->data;

    // 既にautoモードなら何もしない
    if (auto_mode_.load()) return;

    // しきい値を超えたら自動ピックアップモード開始
    // （例：負圧が十分下がる/吸い付いた → ゴミつかんだと判断）
    if (latest_pressure_ < pressure_threshold_) {
      RCLCPP_INFO(this->get_logger(),
        "Pressure %.2f < threshold %.2f → ENTER AUTO PICKUP MODE",
        latest_pressure_, pressure_threshold_);

      auto_mode_.store(true);
      auto_step_ = 0;

      // 状態表示を切り替えたいならここで /robot/state を出す
      std_msgs::msg::String state_msg;
      state_msg.data = "pickup";
      pub_robot_state_->publish(state_msg);
    }
  }

  //========================
  // 自動ピックアップシーケンス送り
  //========================
  void autoSequenceLoop()
  {
    if (!auto_mode_.load()) {
      return;  // まだトリガーされてない
    }

    if (pickup_sequence_.empty()) {
      RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
        "pickup_sequence_ is empty but auto_mode_ is true!");
      return;
    }

    if (auto_step_ >= pickup_sequence_.size()) {
      // すでに最後まで実行済み
      RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
        "AUTO PICKUP MODE finished (holding last pose).");
      return;
    }

    const auto &angles = pickup_sequence_[auto_step_];

    // --- Motor1-9 publish ---
    if (angles.size() < 9) {
      RCLCPP_ERROR(this->get_logger(),
        "pickup_sequence step %zu has <9 values!", auto_step_);
      return;
    }

    std_msgs::msg::Float32MultiArray motor1_9_msg;
    motor1_9_msg.data.assign(angles.begin(), angles.begin() + 9);
    pub_motor1_9_->publish(motor1_9_msg);

    // --- Motor10 publish (★ここを追加した) ---
    // pickup_sequence_[i][9] を motor10 の角度として使う
    if (angles.size() >= 10) {
      std_msgs::msg::Float32 motor10_msg;
      motor10_msg.data = angles[9];
      pub_motor10_->publish(motor10_msg);

      RCLCPP_INFO(this->get_logger(),
        "AUTO PICKUP step %zu/%zu → Pub motor1-9 + motor10(%.2f)",
        auto_step_, pickup_sequence_.size() - 1, motor10_msg.data);
    } else {
      RCLCPP_INFO(this->get_logger(),
        "AUTO PICKUP step %zu/%zu → Pub motor1-9 (no motor10 in data)",
        auto_step_, pickup_sequence_.size() - 1);
    }

    // 次のステップへ
    auto_step_++;

    if (auto_step_ >= pickup_sequence_.size()) {
      RCLCPP_INFO(this->get_logger(),
        "AUTO PICKUP MODE completed. Holding final pose.");

      // 終了状態を出したいならここ
      std_msgs::msg::String state_msg;
      state_msg.data = "retrieving";
      pub_robot_state_->publish(state_msg);
    }
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<AutoSequenceNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
