cmake_minimum_required(VERSION 3.5)
project(Practica2)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(kobuki_ros_interfaces REQUIRED)

add_library(${PROJECT_NAME}
  src/Practica2/BumperLoggerNode.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
target_link_libraries(${PROJECT_NAME})
  ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  geometry_msgs
  kobuki_ros_interfaces
)


add_executable(bumper_logger_node src/bumper_logger_main.cpp)
target_link_libraries(bumper_logger_node ${PROJECT_NAME})


install(TARGETS
  ${PROJECT_NAME}
  bumper_logger_node
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install (DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(
  rclcpp
  geometry_msgs
  kobuki_ros_interfaces
)
ament_package()
