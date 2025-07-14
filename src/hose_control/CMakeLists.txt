cmake_minimum_required(VERSION 3.8)
project(hose_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED) 
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

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
add_executable(nearest_motor_publisher src/nearest_motor_publisher.cpp)
ament_target_dependencies(
  nearest_motor_publisher
  rclcpp
  std_msgs
  geometry_msgs
)
# ─── feedback_goal_position_node ─────────────────
add_executable(feedback_goal_position_node      # ★ add_executable を忘れずに
  src/feedback_goal_position_node.cpp)                    #   ←ファイル名に合わせて変更

ament_target_dependencies(feedback_goal_position_node
  rclcpp std_msgs geometry_msgs)

target_link_libraries(feedback_goal_position_node
  Eigen3::Eigen)                                # Eigen をリンク

install(TARGETS
  nearest_motor_publisher
  feedback_goal_position_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
