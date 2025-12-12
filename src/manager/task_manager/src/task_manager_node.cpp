#include <chrono>
#include <memory>
#include <string>
#include <deque>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>

using namespace std::chrono_literals;

class TaskManagerNode : public rclcpp::Node {
public:
  TaskManagerNode()
  : rclcpp::Node("task_manager")
  {
    RCLCPP_INFO(this->get_logger(), "✅ Task Manager ノード (C++) を起動しました");

    // パラメータ宣言
    this->declare_parameter<double>("initial_angle_deg", 20.0);
    this->declare_parameter<double>("initial_angle_delay_sec", 0.5);
    this->declare_parameter<double>("search_start_delay_sec", 2.0);
    this->declare_parameter<double>("state_publish_period_sec", 1.0);
    this->declare_parameter<double>("stop_duration_sec", 1.0);

    initial_angle_deg_ = this->get_parameter("initial_angle_deg").as_double();
    initial_angle_delay_sec_ = this->get_parameter("initial_angle_delay_sec").as_double();
    search_start_delay_sec_ = this->get_parameter("search_start_delay_sec").as_double();
    state_publish_period_sec_ = this->get_parameter("state_publish_period_sec").as_double();
    stop_duration_sec_ = this->get_parameter("stop_duration_sec").as_double();

    // 状態定義
    STATE_INITIALIZING = "initializing";
    STATE_SEARCHING = "searching";
    STATE_APPROACHING = "approaching";
    STATE_COLLECTING = "collecting";
    STATE_STOPPING = "stopping";

    // Publisher / Subscriber
    state_pub_ = this->create_publisher<std_msgs::msg::String>("/robot/state", 10);
    camera_angle_pub_ = this->create_publisher<std_msgs::msg::Float32>("/cameraswingmotor/target_angle", 10);

    detected_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
      "/detected_depth_points", 10,
      std::bind(&TaskManagerNode::detected_objects_callback, this, std::placeholders::_1));

    hose_result_sub_ = this->create_subscription<std_msgs::msg::Bool>(
      "/hose/result", 10,
      std::bind(&TaskManagerNode::hose_result_callback, this, std::placeholders::_1));

    chaser_completion_sub_ = this->create_subscription<std_msgs::msg::Bool>(
      "/chaser/approach_completed", 10,
      std::bind(&TaskManagerNode::chaser_completion_callback, this, std::placeholders::_1));

    // 状態を定期的に配信するタイマー
    state_publish_timer_ = this->create_wall_timer(
      std::chrono::duration<double>(state_publish_period_sec_),
      std::bind(&TaskManagerNode::publish_state, this));

    // 初期状態
    set_state(STATE_INITIALIZING);

    // カメラ初期化（不要なのでコメントアウト）
    // initialize_camera();

    // 2秒後に探索開始
    transition_timer_ = this->create_wall_timer(
      std::chrono::duration<double>(search_start_delay_sec_),
      std::bind(&TaskManagerNode::start_searching, this));
  }

private:
  // 状態
  std::string STATE_INITIALIZING;
  std::string STATE_SEARCHING;
  std::string STATE_APPROACHING;
  std::string STATE_COLLECTING;
  std::string STATE_STOPPING;

  std::string state_;
  std::deque<std::string> target_queue_;

  // パラメータ値
  double initial_angle_deg_{};
  double initial_angle_delay_sec_{};
  double search_start_delay_sec_{};
  double state_publish_period_sec_{};
  double stop_duration_sec_{};

  // Publishers / Subscribers
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr camera_angle_pub_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr detected_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr hose_result_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr chaser_completion_sub_;

  // Timers
  rclcpp::TimerBase::SharedPtr state_publish_timer_;
  rclcpp::TimerBase::SharedPtr transition_timer_;
  rclcpp::TimerBase::SharedPtr initial_angle_timer_;

  void initialize_camera()
  {
    RCLCPP_INFO(this->get_logger(), "カメラの初期化を行います...");
    initial_angle_timer_ = this->create_wall_timer(
      std::chrono::duration<double>(initial_angle_delay_sec_),
      std::bind(&TaskManagerNode::_publish_initial_camera_angle, this));
  }

  void _publish_initial_camera_angle()
  {
    std_msgs::msg::Float32 msg;
    msg.data = static_cast<float>(initial_angle_deg_);
    camera_angle_pub_->publish(msg);
    RCLCPP_INFO(this->get_logger(), "カメラの角度を %f 度に設定しました。", msg.data);
    if (initial_angle_timer_ && !initial_angle_timer_->is_canceled()) {
      initial_angle_timer_->cancel();
    }
  }

  void set_state(const std::string & new_state)
  {
    if (state_ == new_state) return;
    state_ = new_state;
    RCLCPP_INFO(this->get_logger(), "====== 状態が [ %s ] に遷移しました ======", state_.c_str());
    publish_state();

    if (transition_timer_ && !transition_timer_->is_canceled()) {
      transition_timer_->cancel();
    }

    if (state_ == STATE_APPROACHING) {
      RCLCPP_INFO(this->get_logger(), "  -> ゴミに接近中...ObjectChaserNodeからの完了通知を待ちます。");
      // 必要ならここでナビゲーションへ目標を指示する。
    } else if (state_ == STATE_COLLECTING) {
      RCLCPP_INFO(this->get_logger(), "  -> ホースでゴミを回収中...");
      // ホース制御へ回収開始を指示する。
    } else if (state_ == STATE_STOPPING) {
      RCLCPP_INFO(this->get_logger(), "  -> 一時停止中...");
      transition_timer_ = this->create_wall_timer(
        std::chrono::duration<double>(stop_duration_sec_),
        std::bind(&TaskManagerNode::on_stop_done, this));
    }
  }

  void publish_state()
  {
    std_msgs::msg::String msg;
    msg.data = state_;
    state_pub_->publish(msg);
  }

  void start_searching()
  {
    if (transition_timer_ && !transition_timer_->is_canceled()) {
      transition_timer_->cancel();
    }
    set_state(STATE_SEARCHING);
  }

  void on_stop_done()
  {
    if (transition_timer_ && !transition_timer_->is_canceled()) {
      transition_timer_->cancel();
    }
    if (state_ == STATE_STOPPING) {
      if (!target_queue_.empty()) {
        target_queue_.pop_front();
      }
      if (!target_queue_.empty()) {
        RCLCPP_INFO(this->get_logger(), "次のゴミへ移動します。");
        set_state(STATE_APPROACHING);
      } else {
        RCLCPP_INFO(this->get_logger(), "全てのゴミを回収しました。探索を再開します。");
        set_state(STATE_SEARCHING);
      }
    }
  }

  void detected_objects_callback(const geometry_msgs::msg::PointStamped::SharedPtr /* msg */)
  {
    if (state_ == STATE_SEARCHING) {
      RCLCPP_INFO(this->get_logger(), "🗑️ ゴミを検出しました。タスクキューに追加します。");
      target_queue_.push_back("dummy_target_1");
      set_state(STATE_APPROACHING);
    }
  }

  void chaser_completion_callback(const std_msgs::msg::Bool::SharedPtr msg)
  {
    if (msg->data && state_ == STATE_APPROACHING) {
      RCLCPP_INFO(this->get_logger(), "👍 接近完了通知を受信しました。ホースでの回収を開始します。");
      set_state(STATE_COLLECTING);
    }
  }

  void hose_result_callback(const std_msgs::msg::Bool::SharedPtr msg)
  {
    if (state_ == STATE_COLLECTING) {
      if (msg->data) {
        RCLCPP_INFO(this->get_logger(), "👍 回収成功！");
      } else {
        RCLCPP_INFO(this->get_logger(), "😥 回収失敗...");
      }
      set_state(STATE_STOPPING);
    }
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TaskManagerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
