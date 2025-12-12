cmake_minimum_required(VERSION 3.8)
project(task_manager)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 依存関係
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(hose_control REQUIRED)
find_package(aruco_interfaces REQUIRED)
find_package(target_selector REQUIRED)

# 実行ファイル
add_executable(task_manager_node src/task_manager_node.cpp)
ament_target_dependencies(task_manager_node
  rclcpp
  std_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)
add_executable(movement_controller_node src/movement_controller_node.cpp)
ament_target_dependencies(movement_controller_node
  rclcpp
  std_msgs
  geometry_msgs
)
add_executable(vacuum_manager_node src/vacuum_manager_node.cpp)
ament_target_dependencies(vacuum_manager_node
  rclcpp
  std_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  hose_control
)
add_executable(manipulator_manager_node src/manipulator_manager.cpp)
ament_target_dependencies(manipulator_manager_node
  rclcpp
  std_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  hose_control
  aruco_interfaces
  target_selector
)

# インストール
install(TARGETS
  task_manager_node 
  movement_controller_node
  vacuum_manager_node
  manipulator_manager_node
  DESTINATION lib/${PROJECT_NAME}
)

# install(DIRECTORY launch
#   DESTINATION share/${PROJECT_NAME}
# )

install(FILES package.xml
  DESTINATION share/${PROJECT_NAME}
)

# テスト関連
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
