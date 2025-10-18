#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <cmath>
#include <chrono> // ← 追加
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
    tolerance_   = this->declare_parameter("tolerance", 40.0);
    tolerance10_ = this->declare_parameter("tolerance10", 0.0);
    on_delay_    = this->declare_parameter("on_delay", 5); // 吸引ONまでの遅延時間 (秒)

    flag_pub_ = this->create_publisher<std_msgs::msg::Bool>("/vacuum_flag", 10);

    sub_9_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
      "/motor_current_angles", 10,
      std::bind(&VacuumManagerNode::multi_cb, this, std::placeholders::_1));

    sub_10_ = this->create_subscription<std_msgs::msg::Float32>(
      "/chokudomotor/angle", 10,
      std::bind(&VacuumManagerNode::motor10_cb, this, std::placeholders::_1));

    state_sub_ = this->create_subscription<std_msgs::msg::String>(
      "/robot/state", 10,
      std::bind(&VacuumManagerNode::state_cb, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(), "VacuumManagerNode started (tol = %.2f deg, on_delay = %.2f s)", tolerance_, on_delay_);
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
    check_and_publish();
  }

  void motor10_cb(const std_msgs::msg::Float32::SharedPtr msg)
  {
    latest_10_ = *msg;
    has_10_    = true;
    check_and_publish();
  }

  void state_cb(const std_msgs::msg::String::SharedPtr msg)
  {
    // 状態が "collecting" になったらタイマーを開始する
    if (msg->data == "collecting") {
      // もし既にタイマーが動いていたら、一度キャンセルしてリセットする
      if (suction_on_timer_ && !suction_on_timer_->is_canceled()) {
        suction_on_timer_->cancel();
      }
      on_latched_ = false;  // 新しい吸引サイクルを開始できるようにラッチを解除

      RCLCPP_INFO(get_logger(), "State is 'collecting': Suction will turn ON in %.2f seconds.", on_delay_);

      // 指定時間後に turn_suction_on() を1回だけ呼び出すタイマーを作成
      suction_on_timer_ = this->create_wall_timer(
        std::chrono::duration<double>(on_delay_),
        std::bind(&VacuumManagerNode::turn_suction_on, this));
    }
  }

  // ---------- 判定と処理 ----------
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
    RCLCPP_INFO(get_logger(), "All 10 motors reached stop angles → suction_flag=false");
  }

  void turn_suction_on()
  {
    // タイマーは一度きりなので、キャンセルして止める
    if (suction_on_timer_) {
        suction_on_timer_->cancel();
    }

    // 既にONになっていれば何もしない
    if (on_latched_) {
      return;
    }

    std_msgs::msg::Bool msg;
    msg.data = true;
    flag_pub_->publish(msg);
    on_latched_ = true;
    RCLCPP_INFO(get_logger(), "Timer fired: suction_flag=true");
  }

  // ---------- メンバ ----------
  double tolerance_;
  double tolerance10_;
  double on_delay_; // ← 変更
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr flag_pub_;

  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr sub_9_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr           sub_10_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr            state_sub_;

  rclcpp::TimerBase::SharedPtr suction_on_timer_; // ← 追加

  std_msgs::msg::Float32MultiArray latest_9_;
  std_msgs::msg::Float32           latest_10_;
  bool has_10_{false};
  bool on_latched_{false};

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