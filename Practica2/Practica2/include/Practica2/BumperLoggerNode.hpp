// Copyright 2026 Intelligent Robotics Lab
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef PRACTICA2__BUMPER_LOGGER_NODE_HPP_
#define PRACTICA2__BUMPER_LOGGER_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "kobuki_ros_interfaces/msg/bumper_event.hpp"
#include "geometry_msgs/msg/twist.hpp"

class BumperLoggerNode : public rclcpp::Node
{
public:
  BumperLoggerNode();

private:
  //Callbacks
  void bumper_callback(const kobuki_ros_interfaces::msg::BumperEvent::SharedPtr msg);
  
  void control_timer_callback();

  // Suscripción y publicación
  rclcpp::Subscription<kobuki_ros_interfaces::msg::BumperEvent>::SharedPtr bumper_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  
  // Timer para control a 10 Hz
  rclcpp::TimerBase::SharedPtr control_timer_;

  // Estado interno
  bool left_pressed_;
  bool center_pressed_;
  bool right_pressed_;

  //Parámetros configurables
  //double linear_speed_=0.2;
  //double angular_speed_=0.5;
  double linear_speed_;
  double angular_speed_;
};

#endif // PRACTICA2__BUMPER_LOGGER_NODE_HPP_
