cmake_minimum_required(VERSION 3.8)
project(target_selector)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(rosidl_default_generators REQUIRED)

set(srv_files
  "srv/GetTarget.srv"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${srv_files}
  DEPENDENCIES geometry_msgs
)

add_executable(target_selector_node_exe 
  src/target_selector_node.cpp 
  src/target_selector_core.cpp
)

target_include_directories(target_selector_node_exe PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(target_selector_node_exe 
  rclcpp 
  std_msgs 
  sensor_msgs 
  geometry_msgs 
  tf2 
  tf2_ros
)

# ▼▼▼ add_dependencies を削除し、こちらを追加 ▼▼▼
# 自動生成されたサービスライブラリに実行可能ファイルをリンクする
target_link_libraries(target_selector_node_exe
  ${PROJECT_NAME}__rosidl_typesupport_cpp
)

install(TARGETS
  target_selector_node_exe
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()