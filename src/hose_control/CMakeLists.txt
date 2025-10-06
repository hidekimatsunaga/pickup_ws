cmake_minimum_required(VERSION 3.8)
project(hose_control)

# ────────────────────────────────────────────────────────────────
# 共通オプション
# ────────────────────────────────────────────────────────────────
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ────────────────────────────────────────────────────────────────
# 依存パッケージ
# ────────────────────────────────────────────────────────────────
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(aruco_interfaces REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(target_selector REQUIRED)

# include/ ディレクトリを全ターゲットに公開（方法 A）
include_directories(include)
include_directories(${EIGEN3_INCLUDE_DIRS})

# ────────────────────────────────────────────────────────────────
# 実行ファイル
# ────────────────────────────────────────────────────────────────
add_executable(nearest_motor_publisher src/nearest_motor_publisher.cpp)
ament_target_dependencies(nearest_motor_publisher
  rclcpp std_msgs geometry_msgs)

add_executable(feedback_goal_position_node src/feedback_goal_position_node.cpp)
ament_target_dependencies(feedback_goal_position_node
  rclcpp std_msgs geometry_msgs Eigen3 aruco_interfaces vision_msgs visualization_msgs target_selector)

add_executable(feedback_motor_publisher src/feedback_motor_publisher.cpp)
ament_target_dependencies(feedback_motor_publisher
  rclcpp std_msgs geometry_msgs Eigen3 aruco_interfaces vision_msgs)

add_executable(feedback_m_kg_m src/feedback_m_kg_m.cpp)
ament_target_dependencies(feedback_m_kg_m
  rclcpp std_msgs geometry_msgs Eigen3 aruco_interfaces vision_msgs)

add_executable(flag_manager src/flag_manager.cpp)
ament_target_dependencies(flag_manager
  rclcpp std_msgs geometry_msgs Eigen3 aruco_interfaces vision_msgs)

add_executable(feedback_2_node src/feedback_2_node.cpp)
ament_target_dependencies(feedback_2_node
  rclcpp std_msgs geometry_msgs Eigen3 aruco_interfaces vision_msgs)

add_executable(lookup_table src/lookup_table.cpp)
ament_target_dependencies(lookup_table
  rclcpp std_msgs geometry_msgs Eigen3 aruco_interfaces vision_msgs visualization_msgs)
  
add_executable(lookup_table_extrapolation src/lookup_table_extrapolation.cpp)
ament_target_dependencies(lookup_table_extrapolation
  rclcpp std_msgs geometry_msgs Eigen3 aruco_interfaces vision_msgs visualization_msgs)
  # ────────────────────────────────────────────────────────────────
# インストール設定
# ────────────────────────────────────────────────────────────────
install(TARGETS
  nearest_motor_publisher
  feedback_goal_position_node
  feedback_motor_publisher
  feedback_m_kg_m
  flag_manager
  feedback_2_node
  lookup_table
  lookup_table_extrapolation
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/ DESTINATION include)

ament_package()
