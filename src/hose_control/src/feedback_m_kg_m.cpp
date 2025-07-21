#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_array.hpp> 
#include <Eigen/Dense>                // ★ 追加
#include <cmath>

class FeedbackMKgM : public rclcpp::Node {
public:
  FeedbackMKgM()
  : Node("feedback_m_kg_m"),
    K_(this->declare_parameter("gain", 0.8)),
    tol_(this->declare_parameter("tolerance", 0.01)),
    meas_received_(false)             // ★ 初期化
  {
    // 目標位置
    sub_goal_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
      "/detected_depth_points", 10,
      [this](const geometry_msgs::msg::PointStamped::SharedPtr msg)  // ★ 明示型
      {
        goal_ = *msg;
        publish_cmd();         // 初回送信
      });

    // 実測位置
    sub_meas_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
      "/aruco/poses",           // ★トピック名を合わせる
      10,
      [this](const geometry_msgs::msg::PoseArray::SharedPtr msg)
      {
          if (msg->poses.empty()) return;       // 未検出
          // ここでは先頭のマーカーを使用（id で選ぶなら下で分岐）
          meas_.header = msg->header;
          meas_.point.x = msg->poses[0].position.x;
          meas_.point.y = msg->poses[0].position.y;
          meas_.point.z = msg->poses[0].position.z;
          meas_received_ = true;
          feedback();
      });

    pub_cmd_ = this->create_publisher<geometry_msgs::msg::PointStamped>(
      "/goal_point", 10);
  }

private:
  /* ---------- メンバ ---------- */
  geometry_msgs::msg::PointStamped goal_, meas_, cmd_;
  double K_, tol_;
  bool meas_received_;  // ★ 未受信ガード

  /* ★ ここを追加：サブスクライバのメンバ宣言 */
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr sub_goal_;
  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr sub_meas_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr pub_cmd_;
  

  /* ---------- 関数 ---------- */
  void publish_cmd() {
    cmd_ = goal_;          // 初回は目標そのまま
    pub_cmd_->publish(cmd_);
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

    if (e.norm() < tol_) return;               // 近ければ終了

    Vec3 next = m + K_ * e;                    // 目標補正
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
  auto node = std::make_shared<FeedbackMKgM>();  // ★ クラス名を揃える
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
