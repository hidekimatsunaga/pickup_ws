#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/bool.hpp>
#include <cmath>
#include "hose_control/motor_initial_position.hpp"           // ← 先ほどのヘッダ
#include "hose_control/motor_pickup_position.hpp"

class FlagManager : public rclcpp::Node
{
public:
  FlagManager()
  : Node("flag_manager"),
    stop_angles_(motor_sequences::pickup_sequence.back()),
    stop_motor10_angle_(54.0f) 
  {
    tolerance_       = this->declare_parameter("tolerance", 80.0);     // deg
    tolerance10_     = this->declare_parameter("tolerance10", 80.0);     // deg
    require_arm_     = this->declare_parameter("require_arm", true);  // /start_grasp が必要か
    min_on_interval_ = this->declare_parameter("min_on_interval", 0.5); // ONの連打抑制[s]

    flag_pub_ = this->create_publisher<std_msgs::msg::Bool>("/vacuum_flag", 10);

    // 9 軸の現在角
    sub_9_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
      "/motor_current_angles", 10,
      std::bind(&FlagManager::multi_cb, this, std::placeholders::_1));

    // motor10 の現在角
    sub_10_ = this->create_subscription<std_msgs::msg::Float32>(
      "/chokudomotor/angle", 10,
      std::bind(&FlagManager::motor10_cb, this, std::placeholders::_1));

    // 把持開始の合図（任意）：True が来たら「次の角度受信で一度だけON」
    arm_sub_ = this->create_subscription<std_msgs::msg::Bool>(
      "/start_grasp", 10,
      std::bind(&FlagManager::arm_cb, this, std::placeholders::_1));


    RCLCPP_INFO(get_logger(), "FlagManager started (tol = %.2f deg)", tolerance_);
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
    maybe_publish_on();               // ★ 一度だけON（必要条件を満たしたら）   
    check_and_publish();
  }

  void motor10_cb(const std_msgs::msg::Float32::SharedPtr msg)
  {
    latest_10_ = *msg;
    has_10_    = true;
    maybe_publish_on();               // ★ 一度だけON（必要条件を満たしたら）
    check_and_publish();
  }

  void arm_cb(const std_msgs::msg::Bool::SharedPtr msg)
  {
    if (msg->data) {
      armed_ = true;
      on_latched_ = false;  // 新しい把持サイクル開始
      RCLCPP_INFO(get_logger(), "Armed: will publish ON at next angle update");
    }
  }
  // ---------- 判定 ----------
  void check_and_publish()
  {
    if (!has_10_ || latest_9_.data.empty()) return;   // 情報不足

    // 9 軸判定
    for (size_t i = 0; i < stop_angles_.size(); ++i) {
      if (std::fabs(latest_9_.data[i] - stop_angles_[i]) > tolerance_) return;
    }
    // motor10 判定
    if (std::fabs(latest_10_.data - stop_motor10_angle_) > tolerance10_) return;

    // すべて到達したら OFF
    std_msgs::msg::Bool flag_msg;
    flag_msg.data = false;
    flag_pub_->publish(flag_msg);

    // 一度 OFF したら再度判定したい場合に備え、到達情報をリセット
    latest_9_.data.clear();
    has_10_ = false;
    // 次のサイクルに備えてラッチ解除
    on_latched_ = false;
    armed_ = false;
    RCLCPP_INFO(get_logger(), "All 10 motors reached stop angles → suction_flag=false");
  
  }

  // ---------- 一度だけON発行（ラッチ） ----------
  void maybe_publish_on()
  {
    if (on_latched_) return;                 // すでにON済み
    if (require_arm_ && !armed_) return;     // /start_grasp が必要なら、合図待ち

    const auto now = this->get_clock()->now();
    if (last_on_time_.nanoseconds() != 0) {
      const double dt = (now - last_on_time_).seconds();
      if (dt < min_on_interval_) return;     // 連打防止
    }

    std_msgs::msg::Bool msg;
    msg.data = true;                          // ← ON を 1 回だけ出す
    flag_pub_->publish(msg);
    on_latched_ = true;
    last_on_time_ = now;
    if (require_arm_) armed_ = false;         // 使い切り
    RCLCPP_INFO(get_logger(), "suction_flag=true (one-shot ON)");
  }
  // ---------- メンバ ----------
  double tolerance_;
  double tolerance10_;
  bool   require_arm_;
  double min_on_interval_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr flag_pub_;

  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr sub_9_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr           sub_10_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr              arm_sub_;

  std_msgs::msg::Float32MultiArray latest_9_;
  std_msgs::msg::Float32           latest_10_;
  bool has_10_{false};

  // ラッチ系
  bool on_latched_{false};
  bool armed_{false};
  rclcpp::Time last_on_time_;
  const std::vector<float> stop_angles_;
  const float stop_motor10_angle_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<FlagManager>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}