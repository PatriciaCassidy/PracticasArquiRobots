cmake_minimum_required(VERSION 3.8)
project(patrol_fsm)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(std_msgs REQUIRED)

# Buscar nav2_example instalado en el workspace
find_package(nav2_example REQUIRED)

include_directories(include)

# ── patrol_states_lib ──────────────────────────────────────────
add_library(patrol_states_lib SHARED src/patrol_states.cpp)
ament_target_dependencies(patrol_states_lib
  rclcpp
  rclcpp_action
  nav2_msgs
  nav2_example
  geometry_msgs
  std_msgs
)

# ── patrol_fsm_lib ─────────────────────────────────────────────
add_library(patrol_fsm_lib SHARED src/patrol_fsm_node.cpp)
target_link_libraries(patrol_fsm_lib patrol_states_lib)
ament_target_dependencies(patrol_fsm_lib
  rclcpp
  rclcpp_action
  nav2_msgs
  nav2_example
  geometry_msgs
  std_msgs
)

# ── ejecutable ────────────────────────────────────────────────
add_executable(patrol_fsm src/patrol_fsm_main.cpp)
target_link_libraries(patrol_fsm patrol_fsm_lib patrol_states_lib)
ament_target_dependencies(patrol_fsm
  rclcpp
  rclcpp_action
)

install(TARGETS patrol_states_lib patrol_fsm_lib patrol_fsm
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/ DESTINATION include/)
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()