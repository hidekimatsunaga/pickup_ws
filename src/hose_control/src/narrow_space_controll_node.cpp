#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/float32.hpp>
#include <vector>
#include <string>
#include <cmath>
#include "hose_control/narrow_space_controll_position.hpp"
#include "hose_control/motor_pickup_position.hpp"  

class AutoSequenceNode : public rclcpp::Node {
public:
  AutoSequenceNode()
  : Node("auto_sequence_node"),
    is_in_sequence_mode_(false),
    sequence_step_(0)
  {
    // ヘッダーファイルからシーケンスデータをコピー
    sequence_data_ = motor_sequences::narrow_sequence;

    // --- Subscription ---
    current_angles_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
      "/motor_current_angles", 10,
      std::bind(&AutoSequenceNode::currentAnglesCallback, this, std::placeholders::_1)
    );
    
    // --- Publisher ---
    auto qos = rclcpp::QoS(rclcpp::KeepLast(50)).reliable();
    pub_motor1_9_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("/motor_angles", qos);
    pub_motor10_ = this->create_publisher<std_msgs::msg::Float32>("/chokudomotor/target_angle", 10);

    // ノード起動時にシーケンスを開始
    RCLCPP_INFO(this->get_logger(), "Node started. Automatically starting sequence.");
    is_in_sequence_mode_ = true;
    sequence_step_ = 0;
    
    // std_msgs::msg::Float32 motor10_msg;
    // motor10_msg.data = 54.0f; // float型なので 'f' をつけるのが一般的
    // pub_motor10_->publish(motor10_msg);
    // RCLCPP_INFO(this->get_logger(), "Published initial motor10 angle: %.2f", motor10_msg.data);
    
    publishSequenceStep();
  }

private:
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr current_angles_sub_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pub_motor1_9_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_motor10_;

  bool is_in_sequence_mode_;
  int sequence_step_;
  
  std::vector<std::vector<float>> sequence_data_; 
  std::vector<float> current_motor_angles_;
  
  void currentAnglesCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
  {
    // msg->data は元々 float の vector なので、そのまま代入できます
    current_motor_angles_.assign(msg->data.begin(), msg->data.end());

    if (!is_in_sequence_mode_) return;
    
    const auto& target_angles = sequence_data_[sequence_step_];

    if (isCloseToTarget(target_angles, current_motor_angles_)) {
      RCLCPP_INFO(this->get_logger(), "Sequence step %d reached.", sequence_step_);
      sequence_step_++;
      if (static_cast<size_t>(sequence_step_) >= sequence_data_.size()) {
        RCLCPP_INFO(this->get_logger(), "Sequence finished. Node will be idle.");
        is_in_sequence_mode_ = false;
        return;
      }
      publishSequenceStep();
    }
  }

  // ★★★ 変更点2: シーケンス進行の判定はモーター1-9のみで行うように修正 ★★★
  bool isCloseToTarget(const std::vector<float>& target_with_motor10, const std::vector<float>& current_1_to_9) {
    // targetは10要素、currentは9要素であることを想定
    if (target_with_motor10.size() < 9 || current_1_to_9.size() != 9) {
      RCLCPP_WARN(this->get_logger(), "Size mismatch for comparison. Target size: %zu, Current size: %zu", 
                  target_with_motor10.size(), current_1_to_9.size());
      return false;
    }
    
    float tolerance = 20.0f;
    // モーター1から9までを比較
    for (size_t i = 0; i < 9; ++i) {
      if (std::abs(target_with_motor10[i] - current_1_to_9[i]) > tolerance) return false;
    }
    return true;
  }

    // ★★★ 変更点3: モーター10の指令もパブリッシュするように関数全体を修正 ★★★
  void publishSequenceStep() {
    if(static_cast<size_t>(sequence_step_) >= sequence_data_.size()) return;

    const auto& target_angles_all = sequence_data_[sequence_step_];

    // データが10個あるかチェック
    if (target_angles_all.size() < 10) {
      RCLCPP_ERROR(this->get_logger(), "Sequence data for step %d has less than 10 values!", sequence_step_);
      return;
    }

    RCLCPP_INFO(this->get_logger(), "Publishing sequence step %d.", sequence_step_);

    // --- モーター1-9への指令 ---
    std_msgs::msg::Float32MultiArray motor1_9_msg;
    motor1_9_msg.data.assign(target_angles_all.begin(), target_angles_all.begin() + 9);
    pub_motor1_9_->publish(motor1_9_msg);

    // --- モーター10への指令 ---
    std_msgs::msg::Float32 motor10_msg;
    motor10_msg.data = target_angles_all[9]; // 10番目のデータを格納
    pub_motor10_->publish(motor10_msg);

    RCLCPP_INFO(this->get_logger(), "Published motor 1-9 angles and motor 10 angle: %.2f", motor10_msg.data);
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<AutoSequenceNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}