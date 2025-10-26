#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/bool.hpp>
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
// 初期姿勢定義（ユーザ添付のファイルを参照）
#include "hose_control/motor_initial_position.hpp"

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
  // Publish vacuum flag to allow nodes to turn off suction when sequence completes
  pub_vacuum_flag_ = this->create_publisher<std_msgs::msg::Bool>("/vacuum_flag", 10);

    // --- Subscriber: 気圧センサ(負圧検出) ---
    pressure_sub_ = this->create_subscription<std_msgs::msg::Float32>(
      "/sensor/pressure", 10,
      std::bind(&AutoSequenceNode::pressureCallback, this, std::placeholders::_1)
    );

    // --- Subscriber: 現在角度 (motor1-9 と motor10) を購読してステップ到達判定に使う ---
    sub_motor_current_angles_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
      "/motor_current_angles", 10,
      std::bind(&AutoSequenceNode::motorCurrentAnglesCb, this, std::placeholders::_1)
    );

    sub_motor10_angle_ = this->create_subscription<std_msgs::msg::Float32>(
      "/chokudomotor/angle", 10,
      std::bind(&AutoSequenceNode::motor10AngleCb, this, std::placeholders::_1)
    );

    // パラメータ: ステップ到達判定の閾値とタイムアウト
    step_tolerance_ = this->declare_parameter("step_tolerance", step_tolerance_);
    step_timeout_ = this->declare_parameter("step_timeout", step_timeout_);

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
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr              pub_vacuum_flag_;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr sub_motor_current_angles_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr        sub_motor10_angle_;

  std_msgs::msg::Float32MultiArray latest_9_;
  std_msgs::msg::Float32           latest_10_;
  bool has_9_{false};
  bool has_10_{false};

  // ステップ到達待ちフラグ / 閾値 / タイムアウト
  bool waiting_for_reach_{false};
  double step_tolerance_{30.0};   // deg, デフォルト
  double step_timeout_{10.0};      // 秒, デフォルト
  rclcpp::Time step_start_time_;

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

  // -----------------------
  // 現在角度コールバック
  // -----------------------
  void motorCurrentAnglesCb(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
  {
    latest_9_ = *msg;
    has_9_ = !latest_9_.data.empty();
  }

  void motor10AngleCb(const std_msgs::msg::Float32::SharedPtr msg)
  {
    latest_10_ = *msg;
    has_10_ = true;
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

    // 到達待ちでなければこのステップをpublishして到達待ちに入る
    if (!waiting_for_reach_) {
      // --- Motor1-9 publish ---
      if (angles.size() < 9) {
        RCLCPP_ERROR(this->get_logger(),
          "pickup_sequence step %zu has <9 values!", auto_step_);
        // 警告出して次に進む
        auto_step_++;
        return;
      }

      std_msgs::msg::Float32MultiArray motor1_9_msg;
      motor1_9_msg.data.assign(angles.begin(), angles.begin() + 9);
      pub_motor1_9_->publish(motor1_9_msg);

      // --- Motor10 publish ---
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

      // 到達待ちに入る
      waiting_for_reach_ = true;
      step_start_time_ = this->get_clock()->now();
      return;
    }

    // 到達待ち: 現在角度が目標に近いか確認
    bool reached = true;
    if (angles.size() >= 9) {
      if (!has_9_ || latest_9_.data.size() < 9) reached = false;
      else {
        for (size_t i = 0; i < 9; ++i) {
          if (std::fabs((double)latest_9_.data[i] - (double)angles[i]) > step_tolerance_) {
            reached = false;
            break;
          }
        }
      }
    }
    if (angles.size() >= 10) {
      if (!has_10_) reached = false;
      else if (std::fabs((double)latest_10_.data - (double)angles[9]) > step_tolerance_) reached = false;
    }

    // タイムアウトチェック
    const double elapsed = (this->get_clock()->now() - step_start_time_).seconds();
    if (!reached && elapsed > step_timeout_) {
      RCLCPP_WARN(this->get_logger(),
        "Step %zu not reached within %.2f s → forcing advance (elapsed=%.2f)",
        auto_step_, step_timeout_, elapsed);
      reached = true; // 強制進行
    }

    if (reached) {
      RCLCPP_INFO(this->get_logger(), "Step %zu reached → advance to next step.", auto_step_);
      waiting_for_reach_ = false;
      auto_step_++;

      // 完了判定: 次のステップ番号が配列長を超えたら完了処理
      if (auto_step_ >= pickup_sequence_.size()) {
        RCLCPP_INFO(this->get_logger(), "AUTO PICKUP MODE completed. Holding final pose.");

        // 終了時の処理: 状態通知・吸引OFF・初期姿勢への復帰
        std_msgs::msg::String state_msg;
        state_msg.data = "collecting_finished";
        pub_robot_state_->publish(state_msg);

        std_msgs::msg::Bool vac_msg;
        vac_msg.data = false;
        pub_vacuum_flag_->publish(vac_msg);

        // 初期姿勢へ戻す
        if (stop_angles_.size() >= 9) {
          std_msgs::msg::Float32MultiArray motor1_9_msg;
          motor1_9_msg.data.assign(stop_angles_.begin(), stop_angles_.begin() + 9);
          pub_motor1_9_->publish(motor1_9_msg);
        } else {
          RCLCPP_WARN(this->get_logger(), "motor_initial_position.hpp: stop_angles_ has less than 9 elements");
        }
        std_msgs::msg::Float32 motor10_msg;
        motor10_msg.data = stop_motor10_angle_;
        pub_motor10_->publish(motor10_msg);

        RCLCPP_INFO(this->get_logger(), "Published initial pose (from motor_initial_position.hpp) after AUTO PICKUP completion.");
        auto_mode_.store(false);
      }
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
