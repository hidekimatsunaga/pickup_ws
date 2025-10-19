#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <cmath>
#include <chrono>
#include <hose_control/motor_initial_position.hpp>
#include <hose_control/motor_pickup_position.hpp>
#include "hose_control/narrow_space_controll_position.hpp"

class VacuumManagerNode : public rclcpp::Node
{
public:
  VacuumManagerNode()
  : Node("vacuum_manager_node")
  {
    // --- シーケンスデータから停止角度を取得 ---
    const auto &last_seq = motor_sequences::narrow_sequence.back();

    if (last_seq.size() < 10) {
      RCLCPP_ERROR(this->get_logger(), "Error: last sequence does not have 10 values!");
      throw std::runtime_error("Invalid sequence size");
    }

    // 1〜9軸の停止角度
    stop_angles_.assign(last_seq.begin(), last_seq.begin() + 9);
    // 10軸（直動）の停止角度
    stop_motor10_angle_ = last_seq[9];

    // --- パラメータ ---
    tolerance_   = this->declare_parameter("tolerance", 40.0);
    tolerance10_ = this->declare_parameter("tolerance10", 0.0);
    on_delay_    = this->declare_parameter("on_delay", 5.0);

    // --- Publisher ---
    flag_pub_ = this->create_publisher<std_msgs::msg::Bool>("/vacuum_flag", 10);

    // --- Subscriber ---
    sub_9_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
      "/motor_current_angles", 10,
      std::bind(&VacuumManagerNode::multi_cb, this, std::placeholders::_1));

    sub_10_ = this->create_subscription<std_msgs::msg::Float32>(
      "/chokudomotor/angle", 10,
      std::bind(&VacuumManagerNode::motor10_cb, this, std::placeholders::_1));

    state_sub_ = this->create_subscription<std_msgs::msg::String>(
      "/robot/state", 10,
      std::bind(&VacuumManagerNode::state_cb, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(),
      "VacuumManagerNode started (tol=%.1f, tol10=%.1f, on_delay=%.1fs, stop_motor10=%.2f)",
      tolerance_, tolerance10_, on_delay_, stop_motor10_angle_);
  }

private:
  // --- コールバック ---
  void multi_cb(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
  {
    if (msg->data.size() != stop_angles_.size()) {
      RCLCPP_WARN(get_logger(), "Expected %zu motors but got %zu", stop_angles_.size(), msg->data.size());
      return;
    }
    latest_9_ = *msg;
    check_and_publish();
  }

  void motor10_cb(const std_msgs::msg::Float32::SharedPtr msg)
  {
    latest_10_ = *msg;
    has_10_ = true;
    check_and_publish();
  }

  void state_cb(const std_msgs::msg::String::SharedPtr msg)
  {
    if (msg->data == "collecting") {
      if (suction_on_timer_) suction_on_timer_->cancel();
      on_latched_ = false;

      RCLCPP_INFO(get_logger(), "State='collecting' → suction ON in %.1fs", on_delay_);
      suction_on_timer_ = this->create_wall_timer(
        std::chrono::duration<double>(on_delay_),
        std::bind(&VacuumManagerNode::turn_suction_on, this));
    }
  }

  // --- 判定 ---
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

    RCLCPP_INFO(get_logger(), "All motors reached stop angles → suction_flag=false");
    latest_9_.data.clear();
    has_10_ = false;
    on_latched_ = false;
  }

  void turn_suction_on()
  {
    if (suction_on_timer_) suction_on_timer_->cancel();
    if (on_latched_) return;

    std_msgs::msg::Bool msg;
    msg.data = true;
    flag_pub_->publish(msg);
    on_latched_ = true;
    RCLCPP_INFO(get_logger(), "Timer fired → suction_flag=true");
  }

  // --- メンバ変数 ---
  double tolerance_;
  double tolerance10_;
  double on_delay_;
  std::vector<float> stop_angles_;
  float stop_motor10_angle_;

  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr flag_pub_;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr sub_9_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_10_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr state_sub_;
  rclcpp::TimerBase::SharedPtr suction_on_timer_;

  std_msgs::msg::Float32MultiArray latest_9_;
  std_msgs::msg::Float32 latest_10_;
  bool has_10_{false};
  bool on_latched_{false};
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<VacuumManagerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
