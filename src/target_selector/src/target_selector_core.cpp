#include "target_selector/target_selector_core.hpp"
#include <cmath>
#include <limits>

TargetSelectorNode::TargetSelectorNode() : Node("target_selector_node")
{
  this->declare_parameter<double>("min_z_distance", 0.6);
  min_z_distance_ = this->get_parameter("min_z_distance").as_double();

  // ▼▼▼ ここを修正 ▼▼▼
  // 並列実行が可能な「Reentrant」タイプのコールバックグループを作成
  sub_cbg_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  srv_cbg_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  // ▲▲▲ ここまで ▲▲▲

  // 購読のオプションを作成し、コールバックグループを割り当て
  auto sub_opt = rclcpp::SubscriptionOptions();
  sub_opt.callback_group = sub_cbg_;

  // 作成したオプションを使ってサブスクライバを作成
  subscription_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
    "/detected_depth_points", 10,
    std::bind(&TargetSelectorNode::point_callback, this, std::placeholders::_1),
    sub_opt);

  // サービスサーバーを作成し、コールバックグループを割り当て
  srv_ = this->create_service<target_selector::srv::GetTarget>(
    "get_target",
    std::bind(&TargetSelectorNode::get_target_callback, this, std::placeholders::_1, std::placeholders::_2),
    rmw_qos_profile_services_default,
    srv_cbg_);

  RCLCPP_INFO(this->get_logger(), "目標選択サービスサーバーを起動しました。");
}

// (point_callback と get_target_callback の中身は変更なし)
void TargetSelectorNode::point_callback(const geometry_msgs::msg::PointStamped::SharedPtr msg)
{
  const std::lock_guard<std::mutex> lock(data_mutex_);
  // RCLCPP_INFO(this->get_logger(), "座標受信: X=%.2f, Y=%.2f, Z=%.2f",
  // msg->point.x, msg->point.y, msg->point.z);

  recent_points_.push_back(*msg);
  if (recent_points_.size() > max_points_buffer_) {
    recent_points_.pop_front();
  }
}

void TargetSelectorNode::get_target_callback(
  const std::shared_ptr<target_selector::srv::GetTarget::Request> request,
  std::shared_ptr<target_selector::srv::GetTarget::Response> response)
{
  (void)request;
  RCLCPP_INFO(this->get_logger(), "目標座標のリクエストを受信。有効なデータが見つかるまで待機します...");

  auto start_time = this->now();
  double timeout_seconds = 5.0; // 最大5秒間待機する
  rclcpp::Rate loop_rate(10);   // 1秒間に10回チェックする

  while (rclcpp::ok() && (this->now() - start_time).seconds() < timeout_seconds) {
    std::deque<geometry_msgs::msg::PointStamped> points_to_process;

    { // Mutexで保護されたスコープ
      const std::lock_guard<std::mutex> lock(data_mutex_);
        if (!recent_points_.empty()) {
          points_to_process.push_back(recent_points_.back());
          recent_points_.clear();
        }
    }
    RCLCPP_INFO(this->get_logger(), "バッファサイズ: %zu", recent_points_.size());


    if (!points_to_process.empty()) {
      RCLCPP_INFO(this->get_logger(), "受信データを発見。最適な目標を探索します...");
      
      geometry_msgs::msg::PointStamped closest_point;
      double min_distance = std::numeric_limits<double>::max();
      bool target_found = false;

      for (const auto& point_msg : points_to_process) {
        if (point_msg.point.z < min_z_distance_) {
          continue;
        }

        RCLCPP_INFO(this->get_logger(), "有効候補: X=%.2f, Y=%.2f, Z=%.2f",
          point_msg.point.x, point_msg.point.y, point_msg.point.z);

        double distance = std::sqrt(pow(point_msg.point.x, 2) + pow(point_msg.point.y, 2) + pow(point_msg.point.z, 2));
        if (distance < min_distance) {
          min_distance = distance;
          closest_point = point_msg;
          target_found = true;
        }
      }

      if (target_found) {
        RCLCPP_INFO(this->get_logger(), "最適な目標を発見。座標を返信します。");
        response->success = true;
        response->target_point = closest_point;
        return; // ★成功したので関数を抜ける
      }
    }
    
    loop_rate.sleep(); // 少し待ってからリトライ
  }

  // ループがタイムアウトした場合
  RCLCPP_WARN(this->get_logger(), "タイムアウト: %.1f秒以内に有効な目標が見つかりませんでした。", timeout_seconds);
  response->success = false;
}