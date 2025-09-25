#ifndef TARGET_SELECTOR_CORE_HPP_
#define TARGET_SELECTOR_CORE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "target_selector/srv/get_target.hpp"
#include <deque>
#include <mutex>

class TargetSelectorNode : public rclcpp::Node
{
public:
  TargetSelectorNode();

private:
  void point_callback(const geometry_msgs::msg::PointStamped::SharedPtr msg);
  void get_target_callback(
    const std::shared_ptr<target_selector::srv::GetTarget::Request> request,
    std::shared_ptr<target_selector::srv::GetTarget::Response> response);

  // ▼▼▼ コールバックグループのポインタを追加 ▼▼▼
  rclcpp::CallbackGroup::SharedPtr sub_cbg_;
  rclcpp::CallbackGroup::SharedPtr srv_cbg_;
  // ▲▲▲ ここまで ▲▲▲

  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr subscription_;
  rclcpp::Service<target_selector::srv::GetTarget>::SharedPtr srv_;

  std::deque<geometry_msgs::msg::PointStamped> recent_points_;
  double min_z_distance_;
  size_t max_points_buffer_ = 100;
  std::mutex data_mutex_;
};

#endif // TARGET_SELECTOR_CORE_HPP_