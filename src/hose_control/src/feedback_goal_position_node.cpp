#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <Eigen/Dense>                // ★ 追加
#include <cmath>

class FeedbackGoalPositionNode : public rclcpp::Node {
public:
  FeedbackGoalPositionNode()
  : Node("feedback_goal_position_node"),
    K_(this->declare_parameter("gain", 1.0)),
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
    sub_meas_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
      "/aruco_pose", 10,
      [this](const geometry_msgs::msg::PointStamped::SharedPtr msg)  // ★ 明示型
      {
        meas_ = *msg;
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
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr sub_meas_;

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
