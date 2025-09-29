#include <chrono>
#include <memory>
#include <rclcpp/rclcpp.hpp>

using namespace std::chrono_literals;

class TaskManagerNode : public rclcpp::Node {
public:
  TaskManagerNode() : rclcpp::Node("task_manager") {
    RCLCPP_INFO(this->get_logger(), "task_manager ノードを起動しました");
    timer_ = this->create_wall_timer(1000ms, [this]() {
      RCLCPP_DEBUG(this->get_logger(), "ハートビート");
    });
  }

private:
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TaskManagerNode>());
  rclcpp::shutdown();
  return 0;
}
