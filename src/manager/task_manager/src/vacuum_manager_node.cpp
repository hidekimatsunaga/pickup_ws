#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <iostream>
#include <thread>

class VacuumManagerNode : public rclcpp::Node
{
public:
  VacuumManagerNode()
  : Node("vacuum_manager_node")
  {
    // Publisher
    flag_pub_ = this->create_publisher<std_msgs::msg::Bool>("/vacuum_flag", 10);

    RCLCPP_INFO(this->get_logger(), "VacuumManagerNode started (manual mode only)");

    // --- 手動操作スレッド起動 ---
    input_thread_ = std::thread([this]() { keyboardLoop(); });
    input_thread_.detach();
  }

private:
  void keyboardLoop()
  {
    char key;
    while (rclcpp::ok()) {
      std::cout << "\n[o] ON  [f] OFF  [q] Quit → ";
      std::cin >> key;

      std_msgs::msg::Bool msg;

      if (key == 'o') {
        msg.data = true;
        flag_pub_->publish(msg);
        RCLCPP_INFO(this->get_logger(), "Manual: suction_flag = true (ON)");
      } 
      else if (key == 'f') {
        msg.data = false;
        flag_pub_->publish(msg);
        RCLCPP_INFO(this->get_logger(), "Manual: suction_flag = false (OFF)");
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

  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr flag_pub_;
  std::thread input_thread_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<VacuumManagerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
