#include <chrono>
#include <memory>
#include <string>
#include <algorithm>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/twist.hpp>

using namespace std::chrono_literals;

class MovementControllerNode : public rclcpp::Node {
public:
  MovementControllerNode()
  : rclcpp::Node("movement_controller_node"),
    current_state_(""),
    chaser_timeout_(0.3),
    publish_period_sec_(0.1),
    search_speed_(0.1),
    chaser_active_(false)
  {
    // パラメータ宣言と取得
    this->declare_parameter<double>("publish_period_sec", 0.1);
    this->declare_parameter<double>("chaser_timeout_sec", 0.3);
    this->declare_parameter<double>("search_speed", 0.1);

    publish_period_sec_ = this->get_parameter("publish_period_sec").as_double();
    chaser_timeout_ = this->get_parameter("chaser_timeout_sec").as_double();
    search_speed_ = this->get_parameter("search_speed").as_double();

    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    chaser_cmd_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "/chaser/cmd_vel", 10,
      std::bind(&MovementControllerNode::chaser_cmd_callback, this, std::placeholders::_1));

    state_sub_ = this->create_subscription<std_msgs::msg::String>(
      "/robot/state", 10,
      std::bind(&MovementControllerNode::state_callback, this, std::placeholders::_1));

    publish_timer_ = this->create_wall_timer(
      std::chrono::duration<double>(publish_period_sec_),
      std::bind(&MovementControllerNode::publish_cmd_vel, this));

    last_chaser_time_ = this->get_clock()->now();
    approaching_twist_ = geometry_msgs::msg::Twist();

    RCLCPP_INFO(this->get_logger(), "✅ Movement Controller ノード (C++) を起動しました。");
  }

private:
  // 状態保持
  std::string current_state_;

  // 出力・入力
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr state_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr chaser_cmd_sub_;

  // タイマー
  rclcpp::TimerBase::SharedPtr publish_timer_;

  // chaser指令保持とタイムアウト管理
  geometry_msgs::msg::Twist approaching_twist_;
  rclcpp::Time last_chaser_time_;
  double chaser_timeout_;
  double publish_period_sec_;
  double search_speed_;
  bool chaser_active_;

  void chaser_cmd_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    approaching_twist_ = *msg;
    last_chaser_time_ = this->get_clock()->now();
    chaser_active_ = true;
  }

  void state_callback(const std_msgs::msg::String::SharedPtr msg)
  {
    if (current_state_ != msg->data) {
      std::string upper_state = msg->data;
      std::transform(upper_state.begin(), upper_state.end(), upper_state.begin(), ::toupper);
      RCLCPP_INFO(this->get_logger(), "状態が [ %s ] になりました。移動制御を更新します。", upper_state.c_str());
      current_state_ = msg->data;
    }
  }

  void publish_cmd_vel()
  {
    geometry_msgs::msg::Twist twist;

    const auto now = this->get_clock()->now();
    const double dt = (now - last_chaser_time_).seconds();

    if (current_state_ == "searching") {
      // 探索中：ゆっくり前進
      twist.linear.x = search_speed_;
      twist.angular.z = 0.0;

    } else if (current_state_ == "approaching") {
      // chaser指令が有効かつタイムアウトしていなければ流す
      if (chaser_active_ && dt < chaser_timeout_) {
        twist = approaching_twist_;
      } else {
        twist = geometry_msgs::msg::Twist();
      }

    } else {
      // collecting, stopping, initializing などは停止
      twist = geometry_msgs::msg::Twist();
      approaching_twist_ = geometry_msgs::msg::Twist();
    }

    cmd_vel_pub_->publish(twist);
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MovementControllerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
