#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <Eigen/Dense>
#include <cmath>
#include <std_msgs/msg/bool.hpp>
#include <aruco_interfaces/msg/aruco_markers.hpp>
#include <array>
#include "target_selector/srv/get_target.hpp" // ★ サービスのヘッダーを追加

class FeedbackGoalPositionNode : public rclcpp::Node {
public:
  FeedbackGoalPositionNode()
  : Node("feedback_goal_position_node"),
    K_(this->declare_parameter("gain", 1.0)),
    tol_(this->declare_parameter("tolerance", 0.01)),
    auto_start_(this->declare_parameter("auto_start_grasp", true)),
    arm_err_thresh_(this->declare_parameter("arm_error_threshold", 0.05)),
    meas_received_(false)
  {
    // ★★★ ここから変更 ★★★
    // サービス・クライアントの作成
    client_goal_ = this->create_client<target_selector::srv::GetTarget>("/get_target");

    // auto_startがtrueの場合、1秒後に一度だけサービスを呼び出すタイマーを設定
    if (auto_start_) {
      initial_goal_timer_ = this->create_wall_timer(
        std::chrono::seconds(1),
        std::bind(&FeedbackGoalPositionNode::request_initial_goal, this));
    }
    // ★★★ ここまで変更 ★★★

    // 実測位置 (変更なし)
    sub_meas_ = this->create_subscription<aruco_interfaces::msg::ArucoMarkers>(
      "/aruco/markers", 10,
      [this](const aruco_interfaces::msg::ArucoMarkers::SharedPtr msg) {
        // ... (この中のロジックは変更なし)
        constexpr std::array<int64_t,3> ALLOWED = {0, 1, 2};
        for(size_t i = 0; i < msg->marker_ids.size(); ++i){
          int64_t id = msg->marker_ids[i];
          if(std::find(ALLOWED.begin(), ALLOWED.end(), id) == ALLOWED.end())
          continue;
          if (i >= msg->poses.size()) return;
          meas_.header = msg->header;
          meas_.point.x = msg->poses[i].position.x;
          meas_.point.y = msg->poses[i].position.y;
          meas_.point.z = msg->poses[i].position.z;
          meas_received_ = true;
          feedback();
          return;
        } 
      });

    pub_cmd_ = this->create_publisher<geometry_msgs::msg::PointStamped>("/hose/goal_point", 10);
    pub_start_ = this->create_publisher<std_msgs::msg::Bool>("/start_grasp", 10);
  }

private:
  // ★★★ 追加：サービスを一度だけ呼び出す関数 ★★★
  void request_initial_goal()
  {
    // タイマーは一度しか使わないので停止
    initial_goal_timer_->cancel();

    // サービスサーバーが起動するまで待機
    while (!client_goal_->wait_for_service(std::chrono::seconds(1))) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(this->get_logger(), "クライアントが割り込みを受けました。");
        return;
      }
      RCLCPP_INFO(this->get_logger(), "サービスが利用可能になるのを待っています...");
    }

    // リクエストを作成して非同期で送信
    auto request = std::make_shared<target_selector::srv::GetTarget::Request>();
    client_goal_->async_send_request(
      request,
      [this](rclcpp::Client<target_selector::srv::GetTarget>::SharedFuture future) {
        // 応答を受け取った後の処理
        auto response = future.get();
        if (response->success) {
          RCLCPP_INFO(this->get_logger(), "目標位置を受信しました。");
          goal_ = response->target_point;
          publish_cmd(); // 初回送信
          start_sent_ = false;
        } else {
          RCLCPP_WARN(this->get_logger(), "目標位置の取得に失敗しました。");
        }
      });
  }

  /* ---------- メンバ (一部変更) ---------- */
  geometry_msgs::msg::PointStamped goal_, meas_, cmd_;
  double K_, tol_;
  bool auto_start_;
  double arm_err_thresh_;
  bool meas_received_;
  bool start_sent_{false};

  // ★ サブスクライバを削除し、クライアントとタイマーを追加
  rclcpp::Client<target_selector::srv::GetTarget>::SharedPtr client_goal_;
  rclcpp::TimerBase::SharedPtr initial_goal_timer_;
  
  rclcpp::Subscription<aruco_interfaces::msg::ArucoMarkers>::SharedPtr sub_meas_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr pub_cmd_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_start_;
  

  /* ---------- 関数 (変更なし) ---------- */
  void publish_cmd() {
    cmd_ = goal_;
    pub_cmd_->publish(cmd_);
  }

  void publish_start_grasp_once()
  {
    if (start_sent_ || !auto_start_) return;
    std_msgs::msg::Bool b; b.data = true;
    pub_start_->publish(b);
    start_sent_ = true;
    RCLCPP_INFO(this->get_logger(), "[start_grasp]=true published");
  }

  void feedback() {
    if (!meas_received_) return;
    using Vec3 = Eigen::Vector3d;
    Vec3 g(goal_.point.x, goal_.point.y, goal_.point.z);
    Vec3 m(meas_.point.x, meas_.point.y, meas_.point.z);
    Vec3 e = g - m;

    RCLCPP_INFO(this->get_logger(),
              "誤差: [x=%.4f  y=%.4f  z=%.4f]  |e|=%.4f",
              e.x(), e.y(), e.z(), e.norm());

    if (e.norm() > arm_err_thresh_) {
      publish_start_grasp_once();
    }

    if (e.norm() < tol_) return;

    Vec3 next = g + K_ * e;
    cmd_.header.stamp = this->now();
    cmd_.point.x = next.x();
    cmd_.point.y = next.y();
    cmd_.point.z = next.z();
    pub_cmd_->publish(cmd_);
  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<FeedbackGoalPositionNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}