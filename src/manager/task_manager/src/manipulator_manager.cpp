#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/bool.hpp>
#include <aruco_interfaces/msg/aruco_markers.hpp>
#include "target_selector/srv/get_target.hpp"
#include <chrono>
#include <cmath>

using namespace std::chrono_literals;

// マニピュレータの動作状態を定義
enum class ManipulatorState {
  IDLE,                   // 待機中
  GETTING_TRASH_LOCATION, // ゴミの位置を取得中
  MOVING_TO_TRASH,        // ゴミに向かって移動中
  MOVING_TO_BIN,          // ゴミ箱に向かって移動中
  SEQUENCE_COMPLETE       // シーケンス完了
};

class ManipulatorManager : public rclcpp::Node {
public:
  ManipulatorManager()
  : Node("manipulator_manager")
  {
    // パラメータの宣言
    // ゴミ箱の座標 (x, y, z)
    this->declare_parameter<std::vector<double>>("bin_position", {0.3, 0.0, 0.2});
    // 目標に到達したとみなす距離の閾値 [m]
    this->declare_parameter<double>("goal_tolerance", 0.05);

    // パラメータの取得
    bin_position_ = this->get_parameter("bin_position").as_double_array();
    goal_tolerance_ = this->get_parameter("goal_tolerance").as_double();

    // Publisher
    pub_goal_point_ = this->create_publisher<geometry_msgs::msg::PointStamped>("/hose/goal_point", 10);
    pub_start_grasp_ = this->create_publisher<std_msgs::msg::Bool>("/start_grasp", 10);

    // Subscriber
    // task_managerからの全体状態
    sub_robot_state_ = this->create_subscription<std_msgs::msg::String>(
      "/robot/state", 10, std::bind(&ManipulatorManager::robot_state_callback, this, std::placeholders::_1));
    // アーム先端の現在位置 (ArUcoマーカー)
    sub_aruco_ = this->create_subscription<aruco_interfaces::msg::ArucoMarkers>(
      "/aruco/markers", 10, std::bind(&ManipulatorManager::aruco_callback, this, std::placeholders::_1));

    // Service Client
    client_get_target_ = this->create_client<target_selector::srv::GetTarget>("/get_target");

    // 状態を管理・更新するためのタイマー (10Hz)
    update_timer_ = this->create_wall_timer(100ms, std::bind(&ManipulatorManager::update_state_machine, this));

    RCLCPP_INFO(this->get_logger(), "ManipulatorManager is ready.");
  }

private:
  // --- 状態管理 (ステートマシン) ---
  void update_state_machine() {
    switch (state_) {
      case ManipulatorState::IDLE:
        // 何もせず、/robot/stateからの指令を待つ
        break;

      case ManipulatorState::MOVING_TO_TRASH:
        pub_goal_point_->publish(trash_goal_point_);
        if (is_goal_reached(trash_goal_point_)) {
          RCLCPP_INFO(this->get_logger(), "Reached trash. Moving to bin.");
          state_ = ManipulatorState::MOVING_TO_BIN;
        }
        break;

      case ManipulatorState::MOVING_TO_BIN:
        {
          geometry_msgs::msg::PointStamped bin_goal;
          bin_goal.header.frame_id = "base_link"; // 適切なフレームIDに要変更
          bin_goal.header.stamp = this->now();
          bin_goal.point.x = bin_position_[0];
          bin_goal.point.y = bin_position_[1];
          bin_goal.point.z = bin_position_[2];
          pub_goal_point_->publish(bin_goal);

          if (is_goal_reached(bin_goal)) {
            RCLCPP_INFO(this->get_logger(), "Reached bin. Sequence complete.");
            // FlagManagerがモーター角度を見て自動で吸引を止めることを想定
            // TODO: task_managerに完了報告のTopicをPublishする
            state_ = ManipulatorState::SEQUENCE_COMPLETE;
          }
        }
        break;
      
      case ManipulatorState::GETTING_TRASH_LOCATION:
      case ManipulatorState::SEQUENCE_COMPLETE:
        // これらの状態ではタイマーは何もしない
        break;
    }
  }

  // --- コールバック関数 ---
  void robot_state_callback(const std_msgs::msg::String::SharedPtr msg) {
    // 指令が"collecting"で、現在待機中の場合のみシーケンスを開始
    if (msg->data == "collecting" && state_ == ManipulatorState::IDLE) {
      start_collection_sequence();
    }
  }

  void aruco_callback(const aruco_interfaces::msg::ArucoMarkers::SharedPtr msg) {
    if (!msg->marker_ids.empty()) {
      current_pose_.header = msg->header;
      current_pose_.point = msg->poses[0].position;
      has_received_pose_ = true;
    }
  }

  // --- ヘルパー関数 ---
  void start_collection_sequence() {
    RCLCPP_INFO(this->get_logger(), "Collection sequence started.");
    state_ = ManipulatorState::GETTING_TRASH_LOCATION;

    if (!client_get_target_->wait_for_service(1s)) {
      RCLCPP_ERROR(this->get_logger(), "GetTarget service not available.");
      state_ = ManipulatorState::IDLE; // 待機状態に戻る
      return;
    }

    auto request = std::make_shared<target_selector::srv::GetTarget::Request>();
    client_get_target_->async_send_request(request, 
      [this](rclcpp::Client<target_selector::srv::GetTarget>::SharedFuture future) {
        auto response = future.get();
        if (response->success) {
          RCLCPP_INFO(this->get_logger(), "Received trash location. Moving to trash.");
          trash_goal_point_ = response->target_point;
          
          // 吸引開始をトリガー
          std_msgs::msg::Bool start_msg;
          start_msg.data = true;
          pub_start_grasp_->publish(start_msg);

          state_ = ManipulatorState::MOVING_TO_TRASH;
        } else {
          RCLCPP_WARN(this->get_logger(), "Failed to get trash location.");
          state_ = ManipulatorState::IDLE;
        }
      });
  }

  bool is_goal_reached(const geometry_msgs::msg::PointStamped& goal) {
    if (!has_received_pose_) return false;
    double dx = goal.point.x - current_pose_.point.x;
    double dy = goal.point.y - current_pose_.point.y;
    double dz = goal.point.z - current_pose_.point.z;
    return (std::sqrt(dx*dx + dy*dy + dz*dz) < goal_tolerance_);
  }

  // --- メンバ変数 ---
  ManipulatorState state_{ManipulatorState::IDLE};
  rclcpp::TimerBase::SharedPtr update_timer_;

  // Publishers, Subscribers, Client
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr pub_goal_point_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_start_grasp_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_robot_state_;
  rclcpp::Subscription<aruco_interfaces::msg::ArucoMarkers>::SharedPtr sub_aruco_;
  rclcpp::Client<target_selector::srv::GetTarget>::SharedPtr client_get_target_;

  // データ
  geometry_msgs::msg::PointStamped trash_goal_point_;
  geometry_msgs::msg::PointStamped current_pose_;
  bool has_received_pose_{false};

  // パラメータ
  std::vector<double> bin_position_;
  double goal_tolerance_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ManipulatorManager>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}