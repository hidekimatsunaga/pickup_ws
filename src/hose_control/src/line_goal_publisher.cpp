#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <cmath>
#include <chrono>

using namespace std::chrono_literals;

class LineGoalPublisher : public rclcpp::Node
{
public:
  LineGoalPublisher()
  : Node("line_goal_publisher"), t_(0.0)
  {
    // Publisherの作成
    goal_point_pub_ = this->create_publisher<geometry_msgs::msg::PointStamped>("/hose/goal_point", 10);

    // タイマー（1秒ごとに更新）
    timer_ = this->create_wall_timer(100ms, std::bind(&LineGoalPublisher::timerCallback, this));

    // 直線軌道の始点・終点設定
    start_x_ = -0.3;
    start_y_ = -0.06;
    start_z_ = 0.9;
    end_x_ = 0.3;
    end_y_ = -0.06;
    end_z_ = 0.9;

    RCLCPP_INFO(this->get_logger(), "LineGoalPublisher started. Publishing /hose/goal_point along a straight line.");
  }

private:
  void timerCallback()
  {
    // t_を0〜1の範囲でループ
    t_ += 0.01;
    if (t_ > 1.0) t_ = 0.0;

    // 直線補間 (Lerp)
    geometry_msgs::msg::PointStamped msg;
    msg.header.stamp = this->now();
    msg.header.frame_id = "camera_color_optical_frame";

    msg.point.x = (1 - t_) * start_x_ + t_ * end_x_;
    msg.point.y = (1 - t_) * start_y_ + t_ * end_y_;
    msg.point.z = (1 - t_) * start_z_ + t_ * end_z_;

    goal_point_pub_->publish(msg);

    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                         "Publishing point: x=%.2f, y=%.2f, z=%.2f", 
                         msg.point.x, msg.point.y, msg.point.z);
  }

  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr goal_point_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  double t_;

  // 始点・終点
  double start_x_, start_y_, start_z_;
  double end_x_, end_y_, end_z_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<LineGoalPublisher>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
