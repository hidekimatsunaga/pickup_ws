#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>                       // ← 変更: String型を使うために追加
#include <cmath>
#include <hose_control/motor_initial_position.hpp>
#include <hose_control/motor_pickup_position.hpp>

class VacuumManagerNode : public rclcpp::Node
{
public:
  VacuumManagerNode()
  : Node("vacuum_manager_node"),
    stop_angles_(motor_sequences::pickup_sequence.back()),
    stop_motor10_angle_(54.0f)
  {
    tolerance_       = this->declare_parameter("tolerance", 80.0);
    tolerance10_     = this->declare_parameter("tolerance10", 80.0);
    min_on_interval_ = this->declare_parameter("min_on_interval", 0.5);

    flag_pub_ = this->create_publisher<std_msgs::msg::Bool>("/vacuum_flag", 10);

    sub_9_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
      "/motor_current_angles", 10,
      std::bind(&VacuumManagerNode::multi_cb, this, std::placeholders::_1));

    sub_10_ = this->create_subscription<std_msgs::msg::Float32>(
      "/chokudomotor/angle", 10,
      std::bind(&VacuumManagerNode::motor10_cb, this, std::placeholders::_1));

    // ← 変更: /start_grasp の代わりに /robot/state を購読する
    state_sub_ = this->create_subscription<std_msgs::msg::String>(
      "/robot/state", 10,
      std::bind(&VacuumManagerNode::state_cb, this, std::placeholders::_1));


    RCLCPP_INFO(get_logger(), "VacuumManagerNode started (tol = %.2f deg)", tolerance_);
  }

private:
  // ---------- コールバック ----------
  void multi_cb(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
  {
    if (msg->data.size() != stop_angles_.size()) {
      RCLCPP_WARN(get_logger(), "期待した 9 軸ではありません");
      return;
    }
    latest_9_ = *msg;
    maybe_publish_on();
    check_and_publish();
  }

  void motor10_cb(const std_msgs::msg::Float32::SharedPtr msg)
  {
    latest_10_ = *msg;
    has_10_    = true;
    maybe_publish_on();
    check_and_publish();
  }

  // ← 変更: arm_cb の代わりに state_cb を実装
  void state_cb(const std_msgs::msg::String::SharedPtr msg)
  {
    // 状態が "collecting" になったら吸引ONの準備をする
    if (msg->data == "collecting") {
      armed_ = true;
      on_latched_ = false;  // 新しい吸引サイクルを開始できるようにラッチを解除
      RCLCPP_INFO(get_logger(), "State is 'collecting': Armed for suction ON");
    }
  }

  // ---------- 判定 ----------
  void check_and_publish()
  {
    if (!has_10_ || latest_9_.data.empty()) return;

    for (size_t i = 0; i < stop_angles_.size(); ++i) {
      if (std::fabs(latest_9_.data[i] - stop_angles_[i]) > tolerance_) return;
    }
    if (std::fabs(latest_10_.data - stop_motor10_angle_) > tolerance10_) return;

    std_msgs::msg::Bool flag_msg;
    flag_msg.data = false;
    flag_pub_->publish(flag_msg);

    latest_9_.data.clear();
    has_10_ = false;
    on_latched_ = false;
    armed_ = false;
    RCLCPP_INFO(get_logger(), "All 10 motors reached stop angles → suction_flag=false");
  }

  void maybe_publish_on()
  {
    if (on_latched_ || !armed_) return; // ON済みか、準備ができていなければ何もしない

    const auto now = this->get_clock()->now();
    if (last_on_time_.nanoseconds() != 0) {
      const double dt = (now - last_on_time_).seconds();
      if (dt < min_on_interval_) return;
    }

    std_msgs::msg::Bool msg;
    msg.data = true;
    flag_pub_->publish(msg);
    on_latched_ = true;
    last_on_time_ = now;
    armed_ = false; // トリガーは使い切り
    RCLCPP_INFO(get_logger(), "suction_flag=true (one-shot ON)");
  }

  // ---------- メンバ ----------
  double tolerance_;
  double tolerance10_;
  double min_on_interval_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr flag_pub_;

  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr sub_9_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr           sub_10_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr            state_sub_; // ← 変更

  std_msgs::msg::Float32MultiArray latest_9_;
  std_msgs::msg::Float32           latest_10_;
  bool has_10_{false};

  bool on_latched_{false};
  bool armed_{false};
  rclcpp::Time last_on_time_;
  const std::vector<float> stop_angles_;
  const float stop_motor10_angle_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<VacuumManagerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}