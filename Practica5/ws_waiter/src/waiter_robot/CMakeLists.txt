cmake_minimum_required(VERSION 3.8)
project(waiter_robot)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# --- dependencias -----------------------------------------------------------
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(tf2 REQUIRED)          
find_package(tf2_ros REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(behaviortree_cpp REQUIRED)

include_directories(include)

# --- libreria de nodos BT ---------------------------------------------------
add_library(waiter_nodes SHARED
  src/navigate_to_pose_action.cpp
  src/say_text_action.cpp
  src/listen_text_action.cpp
  src/extract_info_action.cpp
  src/text_utils.cpp
  src/bt_nodes/bt_node_registration.cpp
)

ament_target_dependencies(waiter_nodes
  rclcpp
  rclcpp_action
  std_msgs
  geometry_msgs
  tf2
  tf2_ros
  nav2_msgs
  behaviortree_cpp
)

# --- ejecutable principal ---------------------------------------------------
add_executable(waiter_bt_example src/waiter_bt_example.cpp)
target_link_libraries(waiter_bt_example waiter_nodes)   # FIX: quitar rclcpp::rclcpp redundante
ament_target_dependencies(waiter_bt_example
  rclcpp
  rclcpp_action
  std_msgs
  ament_index_cpp
  behaviortree_cpp
)

# --- install ----------------------------------------------------------------
install(TARGETS waiter_nodes
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS waiter_bt_example
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config launch maps rviz
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()