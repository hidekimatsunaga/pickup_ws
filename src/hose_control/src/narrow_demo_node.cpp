#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/float32.hpp>
#include <vector>
#include <string>
#include <cmath>
#include <thread>
#include <iostream>
#include "hose_control/narrow_space_controll_position.hpp"
#include "hose_control/motor_pickup_position.hpp"

class AutoSequenceNode : public rclcpp::Node {
public:
  AutoSequenceNode()
  : Node("auto_sequence_node"),
    sequence_step_(0)
  {
    // ヘッダーファイルからシーケンスデータをコピー
    sequence_data_ = motor_sequences::narrow_sequence;

    // --- Publisher ---
    auto qos = rclcpp::QoS(rclcpp::KeepLast(50)).reliable();
    pub_motor1_9_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("/motor_angles", qos);
    pub_motor10_ = this->create_publisher<std_msgs::msg::Float32>("/chokudomotor/target_angle", 10);

    RCLCPP_INFO(this->get_logger(), "Node started. Press keys: [n] next, [b] back, [q] quit");

    // キーボード入力監視スレッドを開始
    input_thread_ = std::thread([this]() { this->keyboardLoop(); });
    input_thread_.detach();
  }

private:
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pub_motor1_9_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_motor10_;
  std::vector<std::vector<float>> sequence_data_;
  int sequence_step_;
  std::thread input_thread_;

  void keyboardLoop() {
    char input;
    while (rclcpp::ok()) {
      std::cout << "\n[n] Next  [b] Back  [q] Quit  → ";
      std::cin >> input;

      if (input == 'n') {
        publishSequenceStep(+1);
      } else if (input == 'b') {
        publishSequenceStep(-1);
      } else if (input == 'q') {
        RCLCPP_INFO(this->get_logger(), "Exiting program...");
        rclcpp::shutdown();
        break;
      } else {
        std::cout << "Invalid key. Use [n], [b], or [q]." << std::endl;
      }
    }
  }

  void publishSequenceStep(int direction) {
    // direction = +1: forward, -1: backward
    sequence_step_ += direction;

    // 範囲チェック
    if (sequence_step_ < 0) {
      sequence_step_ = 0;
      RCLCPP_WARN(this->get_logger(), "Already at the first step.");
      return;
    }
    if (static_cast<size_t>(sequence_step_) >= sequence_data_.size()) {
      sequence_step_ = sequence_data_.size() - 1;
      RCLCPP_WARN(this->get_logger(), "Already at the last step.");
      return;
    }

    const auto& target_angles_all = sequence_data_[sequence_step_];
    if (target_angles_all.size() < 10) {
      RCLCPP_ERROR(this->get_logger(), "Sequence step %d has less than 10 values!", sequence_step_);
      return;
    }

    RCLCPP_INFO(this->get_logger(), "Publishing sequence step %d.", sequence_step_);

    // モーター1-9
    std_msgs::msg::Float32MultiArray motor1_9_msg;
    motor1_9_msg.data.assign(target_angles_all.begin(), target_angles_all.begin() + 9);
    pub_motor1_9_->publish(motor1_9_msg);

    // モーター10
    std_msgs::msg::Float32 motor10_msg;
    motor10_msg.data = target_angles_all[9];
    pub_motor10_->publish(motor10_msg);

    RCLCPP_INFO(this->get_logger(), "Published motor 1-9 and motor 10 (%.2f)", motor10_msg.data);
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<AutoSequenceNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
