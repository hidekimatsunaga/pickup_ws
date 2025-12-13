#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <std_msgs/msg/string.hpp>
#include <Eigen/Dense>
#include <cmath>
#include <std_msgs/msg/bool.hpp>
#include <aruco_interfaces/msg/aruco_markers.hpp>
#include <array>


class Feedback2Node : public rclcpp::Node {
public:
  Feedback2Node()
  : Node("feedback_2_node"),
    K_(this->declare_parameter("gain", 1.0)),
    tol_(this->declare_parameter("tolerance", 0.01)),
    auto_start_(this->declare_parameter("auto_start_grasp", true)),
    arm_err_thresh_(this->declare_parameter("arm_error_threshold", 0.05)),
    meas_received_(false),
    current_robot_state_("")
  {
    // ロボット状態購読
    sub_state_ = this->create_subscription<std_msgs::msg::String>(
      "/robot/state", 10,
      [this](const std_msgs::msg::String::SharedPtr msg)
      {
        current_robot_state_ = msg->data;
        RCLCPP_INFO(this->get_logger(), "Robot state: %s", current_robot_state_.c_str());
      });

    // 目標位置
    sub_goal_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
      "/detected_depth_points", 10,
      [this](const geometry_msgs::msg::PointStamped::SharedPtr msg)
      {
        goal_ = *msg;
        // ArUcoがまだ見えていない場合は、取得した検出点をそのまま出力しておく
        if (current_robot_state_ == "collecting" && !meas_received_) {
          publish_cmd();
          RCLCPP_INFO(this->get_logger(), "No ArUco yet; publishing detected point as /hose/goal_point");
        }
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
      "/hose/goal_point", 10);
    pub_start_ = this->create_publisher<std_msgs::msg::Bool>("/start_grasp", 10); // ★ 追加


  }


private:
  /* ---------- メンバ ---------- */
  geometry_msgs::msg::PointStamped goal_, meas_, cmd_;
  double K_, tol_;
  bool auto_start_;
  double arm_err_thresh_;
  bool meas_received_;
  bool start_sent_{false};
  std::string current_robot_state_;


  /* サブスクライバとパブリッシャーのメンバ宣言 */
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_state_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr sub_goal_;
  rclcpp::Subscription<aruco_interfaces::msg::ArucoMarkers>::SharedPtr sub_meas_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr pub_cmd_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_start_;
  


  void publish_cmd() {
    cmd_ = goal_;
    pub_cmd_->publish(cmd_);
  }

  // 一度だけ /start_grasp を出す
  void publish_start_grasp_once()
  {
    if (start_sent_ || !auto_start_) return;
    if (current_robot_state_ != "collecting") return;
    std_msgs::msg::Bool b; b.data = true;
    pub_start_->publish(b);
    start_sent_ = true;
    RCLCPP_INFO(this->get_logger(), "[start_grasp]=true published");
  }


  void feedback() {
    // "collecting" 状態でのみ動作
    if (current_robot_state_ != "collecting") return;
    if (!meas_received_) return;
    using Vec3 = Eigen::Vector3d;
    Vec3 g(goal_.point.x, goal_.point.y, goal_.point.z);
    Vec3 m(meas_.point.x, meas_.point.y, meas_.point.z);
    Vec3 e = g - m;

    RCLCPP_INFO(this->get_logger(),
              "誤差: [x=%.4f  y=%.4f  z=%.4f]  |e|=%.4f",
              e.x(), e.y(), e.z(), e.norm());

    // ゴールに向かう必要がある（十分離れている）ときに/start_graspを一度だけ出す
    if (e.norm() > arm_err_thresh_) {
      publish_start_grasp_once();
    }

    if (e.norm() < tol_) return;


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
  auto node = std::make_shared<Feedback2Node>();  // ★ クラス名を揃える
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
