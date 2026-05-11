cmake_minimum_required(VERSION 3.8)
project(waiter_robot)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(simple_hri_interfaces REQUIRED)
find_package(hri_example REQUIRED)

# Include directories
include_directories(include)

# BT node implementations library
add_library(waiter_nodes SHARED
  src/navigate_to_pose_action.cpp
  src/say_text_client_action.cpp
  src/listen_text_client_action.cpp
  src/extract_info_client_action.cpp
  src/text_utils.cpp
  src/bt_nodes/bt_node_registration.cpp
)

ament_target_dependencies(waiter_nodes
  rclcpp
  rclcpp_action
  std_msgs
  std_srvs
  sensor_msgs
  geometry_msgs
  tf2_ros
  hri_example
  nav2_msgs
  behaviortree_cpp
  simple_hri_interfaces
)

include_directories(include/waiter_robot/bt_nodes)

# Waiter BT example
add_executable(waiter_bt_example src/waiter_bt_example.cpp)
target_link_libraries(waiter_bt_example waiter_nodes)
ament_target_dependencies(waiter_bt_example
  rclcpp
  rclcpp_action
  std_msgs
  ament_index_cpp
  behaviortree_cpp
  simple_hri_interfaces
  hri_example
)

# Install library
install(TARGETS waiter_nodes
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install executables
install(TARGETS
  waiter_bt_example
  DESTINATION lib/${PROJECT_NAME}
)

# Install config files
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

# Install maps
install(DIRECTORY
  maps
  DESTINATION share/${PROJECT_NAME}
)

# Install rviz
install(DIRECTORY
  rviz
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()