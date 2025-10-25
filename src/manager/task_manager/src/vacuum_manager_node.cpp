#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <iostream>
#include <thread>
#include <atomic>

class VacuumManagerNode : public rclcpp::Node
{
public:
  VacuumManagerNode()
  : Node("vacuum_manager_node"),
    current_flag_(false)
  {
    // Publisher (/vacuum_flag)
    flag_pub_ = this->create_publisher<std_msgs::msg::Bool>("/vacuum_flag", 10);

    // Subscriber (/robot/state)
    state_sub_ = this->create_subscription<std_msgs::msg::String>(
      "/robot/state",
      10,
      std::bind(&VacuumManagerNode::stateCallback, this, std::placeholders::_1)
    );

    RCLCPP_INFO(this->get_logger(),
      "VacuumManagerNode started.\n"
      " - manual keys: [o]=ON [f]=OFF [q]=Quit\n"
      " - auto: /robot/state == \"collecting\" → ON");

    // 手動操作スレッド開始
    input_thread_ = std::thread([this]() { keyboardLoop(); });
    input_thread_.detach();
  }

private:
  // -----------------------
  // コールバック: /robot/state
  // -----------------------
  void stateCallback(const std_msgs::msg::String::SharedPtr msg)
  {
    // もし "collecting" が来たら自動でONにする
    if (msg->data == "collecting") {
      RCLCPP_INFO(this->get_logger(),
        "Received /robot/state=\"%s\" → Auto suction ON", msg->data.c_str());
      setVacuumFlag(true, /*is_auto=*/true);
    } else {
      // collecting 以外のstateが来ても OFF にはしない仕様
      RCLCPP_DEBUG(this->get_logger(),
        "Received /robot/state=\"%s\" (no action)", msg->data.c_str());
    }
  }

  // -----------------------
  // 手動入力ループ
  // -----------------------
  void keyboardLoop()
  {
    char key;
    while (rclcpp::ok()) {
      std::cout << "\n[o] ON  [f] OFF  [q] Quit → ";
      std::cin >> key;

      if (key == 'o') {
        setVacuumFlag(true, /*is_auto=*/false);
      }
      else if (key == 'f') {
        setVacuumFlag(false, /*is_auto=*/false);
      }
      else if (key == 'q') {
        RCLCPP_INFO(this->get_logger(), "Manual exit requested.");
        rclcpp::shutdown();
        break;
      }
      else {
        std::cout << "Invalid input. Use [o], [f], or [q]." << std::endl;
      }
    }
  }

  // -----------------------
  // 実際に /vacuum_flag をpublishする共通関数
  // -----------------------
  void setVacuumFlag(bool on, bool is_auto)
  {
    // すでに同じ状態なら無駄に連投しないようにする（スパム防止）
    if (current_flag_.load() == on) {
      if (is_auto) {
        RCLCPP_INFO(this->get_logger(),
          "Vacuum already %s (auto request ignored).",
          on ? "ON" : "OFF");
      } else {
        RCLCPP_INFO(this->get_logger(),
          "Vacuum already %s.",
          on ? "ON" : "OFF");
      }
      return;
    }

    // Publish Bool
    std_msgs::msg::Bool msg;
    msg.data = on;
    flag_pub_->publish(msg);
    current_flag_.store(on);

    if (is_auto) {
      RCLCPP_INFO(this->get_logger(),
        "Auto: suction_flag = %s", on ? "true (ON)" : "false (OFF)");
    } else {
      RCLCPP_INFO(this->get_logger(),
        "Manual: suction_flag = %s", on ? "true (ON)" : "false (OFF)");
    }
  }

  // -----------------------
  // メンバ
  // -----------------------
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr flag_pub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr state_sub_;
  std::thread input_thread_;

  // いま吸ってるかどうかを記憶して、同じ指示を無限に投げ続けないようにする
  std::atomic<bool> current_flag_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<VacuumManagerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
