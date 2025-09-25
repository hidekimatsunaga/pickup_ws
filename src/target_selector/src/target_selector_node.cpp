#include "target_selector/target_selector_core.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<TargetSelectorNode>();

  // ▼▼▼ ここから変更 ▼▼▼
  // 2つのスレッドを持つマルチスレッドエグゼキュータを作成
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  // ▲▲▲ ここまで ▲▲▲

  rclcpp::shutdown();
  return 0;
}