#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_array.hpp> 
#include <Eigen/Dense>                // ★ 追加
#include <cmath>
#include <std_msgs/msg/bool.hpp>
#include <aruco_interfaces/msg/aruco_markers.hpp> // ★ 追加
#include <array>

class FeedbackGoalPositionNode : public rclcpp::Node {
public:
  FeedbackGoalPositionNode()
  : Node("feedback_goal_position_node"),
    K_(this->declare_parameter("gain", 1.0)),
    tol_(this->declare_parameter("tolerance", 0.01)),
    auto_start_(this->declare_parameter("auto_start_grasp", true)),
    arm_err_thresh_(this->declare_parameter("arm_error_threshold", 0.05)),
    meas_received_(false)             // ★ 初期化
  {
    // 目標位置
    sub_goal_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
      "/detected_depth_points", 10,
      [this](const geometry_msgs::msg::PointStamped::SharedPtr msg)  // ★ 明示型
      {
        goal_ = *msg;
        publish_cmd();         // 初回送信
        start_sent_ = false;
      });

    // 実測位置
    sub_meas_ = this->create_subscription<aruco_interfaces::msg::ArucoMarkers>(
      "/aruco/markers",           // ★トピック名を合わせる
      10,
      [this](const aruco_interfaces::msg::ArucoMarkers::SharedPtr msg)
      {
        constexpr std::array<int64_t,3> ALLOWED = {0, 1, 2};

        for(size_t i = 0; i < msg->marker_ids.size(); ++i){
          int64_t id = msg->marker_ids[i];
          if(std::find(ALLOWED.begin(), ALLOWED.end(), id) == ALLOWED.end())
          continue;  // id が許可されていない場合はスキップ

          if (i >= msg->poses.size()) return;       // 念の為の境界確認
          // ここでは先頭のマーカーを使用（id で選ぶなら下で分岐）
          meas_.header = msg->header;
          meas_.point.x = msg->poses[i].position.x;
          meas_.point.y = msg->poses[i].position.y;
          meas_.point.z = msg->poses[i].position.z;
          meas_received_ = true;
          feedback();
          return;
        } 
      });

    pub_cmd_ = this->create_publisher<geometry_msgs::msg::PointStamped>(
      "/goal_point", 10);
    pub_start_ = this->create_publisher<std_msgs::msg::Bool>("/start_grasp", 10); // ★ 追加

  }

private:
  /* ---------- メンバ ---------- */
  geometry_msgs::msg::PointStamped goal_, meas_, cmd_;
  double K_, tol_;
  bool auto_start_;                // ★ 自動把持開始
  double arm_err_thresh_;          // ★ 把持開始の誤差しき
  bool meas_received_;  // ★ 未受信ガード
  bool start_sent_{false};         // ★ このゴールに対して/start_graspをもう出したか

  /* ★ ここを追加：サブスクライバのメンバ宣言 */
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr sub_goal_;
  rclcpp::Subscription<aruco_interfaces::msg::ArucoMarkers>::SharedPtr sub_meas_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr pub_cmd_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_start_; // ★ 把持開始の合図
  

  /* ---------- 関数 ---------- */
  void publish_cmd() {
    cmd_ = goal_;          // 初回は目標そのまま
    pub_cmd_->publish(cmd_);
  }

  // ★ 一度だけ /start_grasp を出す
  void publish_start_grasp_once()
  {
    if (start_sent_ || !auto_start_) return;
    std_msgs::msg::Bool b; b.data = true;
    pub_start_->publish(b);
    start_sent_ = true;
    RCLCPP_INFO(this->get_logger(), "[start_grasp]=true published");
  }

  void feedback() {
    if (!meas_received_) return;               // 実測がまだ来ていない
    using Vec3 = Eigen::Vector3d;
    Vec3 g(goal_.point.x, goal_.point.y, goal_.point.z);
    Vec3 m(meas_.point.x, meas_.point.y, meas_.point.z);
    Vec3 e = g - m;

    // ── 誤差を出力 ─────────────────────────────
    RCLCPP_INFO(this->get_logger(),
              "誤差: [x=%.4f  y=%.4f  z=%.4f]  |e|=%.4f",
              e.x(), e.y(), e.z(), e.norm());
    // ──────────────────────────────────────────
    //ゴールに向かう必要がある　（十分離れている）ときに/start_graspを一度だけ出す
    if (e.norm() > arm_err_thresh_) {
      publish_start_grasp_once();  // ★ 一度だけ把持開始の合図
    }

    if (e.norm() < tol_) return;               // 近ければ終了

    Vec3 next = g + K_ * e;                    // 目標補正
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
  auto node = std::make_shared<FeedbackGoalPositionNode>();  // ★ クラス名を揃える
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
