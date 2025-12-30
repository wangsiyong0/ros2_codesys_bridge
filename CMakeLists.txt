cmake_minimum_required(VERSION 3.5)
project(ros2_codesys_bridge)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 寻找依赖包
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)

# 包含头文件
include_directories(
  include
)

# 构建可执行文件
add_executable(bridge_node
  src/bridge_node.cpp
  src/shared_memory.cpp
)

# 链接依赖库
ament_target_dependencies(bridge_node
  rclcpp
  std_msgs
  nav_msgs
  geometry_msgs
  tf2_ros
  tf2
)

# 安装可执行文件
install(TARGETS
  bridge_node
  DESTINATION lib/${PROJECT_NAME})

# 安装launch文件
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME})

# 测试配置
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()