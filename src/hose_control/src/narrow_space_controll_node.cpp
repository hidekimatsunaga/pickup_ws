#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/float32.hpp>
#include <vector>
#include <string>
#include <cmath>

// ご提示いただいたヘッダーファイルをインクルードします
#include "motor_sequence.hpp" 

class AutoSequenceNode : public rcppl::Node {
public:
  AutoSequenceNode()
  : Node("auto_sequence_node"),
    is_in_sequence_mode_(false),
    sequence_step_(0)
  {
    // ヘッダーファイルからシーケンスデータをコピー
    sequence_data_ = motor_sequences::pickup_sequence;

    // --- Subscription ---
    current_angles_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
      "/motor_current_angles", 10,
      std::bind(&AutoSequenceNode::currentAnglesCallback, this, std::placeholders::_1)
    );
    
    // --- Publisher ---
    auto qos = rclcpp::QoS(rclcpp::KeepLast(50)).reliable();
    pub_motor1_9_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("/motor_angles", qos);
    pub_motor10_ = this->create_publisher<std_msgs/msg/Float32>("/chokudomotor/target_angle", 10);

    // ノード起動時にシーケンスを開始
    RCLCPP_INFO(this->get_logger(), "Node started. Automatically starting sequence.");
    is_in_sequence_mode_ = true;
    sequence_step_ = 0;
    
    std_msgs::msg::Float32 motor10_msg;
    motor10_msg.data = 54.0f; // float型なので 'f' をつけるのが一般的
    pub_motor10_->publish(motor10_msg);
    RCLCPP_INFO(this->get_logger(), "Published initial motor10 angle: %.2f", motor10_msg.data);
    
    publishSequenceStep();
  }

private:
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr current_angles_sub_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pub_motor1_9_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_motor10_;

  bool is_in_sequence_mode_;
  int sequence_step_;
  
  // ★★★ 変更点1: メンバー変数の型を `float` に統一 ★★★
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

  // ★★★ 変更点2: 関数の引数も `float` に統一 ★★★
  bool isCloseToTarget(const std::vector<float>& target, const std::vector<float>& current) {
    if (target.size() != current.size()) return false;
    
    float tolerance = 20.0f; // 許容誤差も float 型に
    for (size_t i = 0; i < target.size(); ++i) {
      // std::abs は float 型にも対応しています
      if (std::abs(target[i] - current[i]) > tolerance) return false;
    }
    return true;
  }

  void publishSequenceStep() {
    if(static_cast<size_t>(sequence_step_) >= sequence_data_.size()) return;

    RCLCPP_INFO(this->get_logger(), "Publishing sequence step %d.", sequence_step_);
    std_msgs::msg::Float32MultiArray angle_msg;

    // ★★★ 変更点3: target_angles は既に float の vector なので、そのまま代入できる ★★★
    angle_msg.data = sequence_data_[sequence_step_];
    
    pub_motor1_9_->publish(angle_msg);
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<AutoSequenceNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}