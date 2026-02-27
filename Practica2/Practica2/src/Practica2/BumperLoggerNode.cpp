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

#include "rclcpp/rclcpp.hpp"
#include "Practica2/BumperLoggerNode.hpp"

using namespace std::chrono_literals;

BumperLoggerNode::BumperLoggerNode()
: Node("bumper_logger_node"),
  left_pressed_(false),
  center_pressed_(false),
  right_pressed_(false)
{
  // Declarar parámetros
  this->declare_parameter<double>("linear_speed", 0.2);
  this->declare_parameter<double>("angular_speed", 0.5);

  // Obtener valores de parámetros
  linear_speed_ = this->get_parameter("linear_speed").as_double();
  angular_speed_ = this->get_parameter("angular_speed").as_double();

  // Crear subscriptor al bumper
  bumper_sub_ = this->create_subscription<kobuki_ros_interfaces::msg::BumperEvent>(
    "/events/bumper", 10,
    std::bind(&BumperLoggerNode::bumper_callback, this, std::placeholders::_1));

  // Crear publicador de velocidades
  cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

  // Crear timer para control a 10 Hz
  control_timer_ = this->create_wall_timer(
    100ms, std::bind(&BumperTeleopNode::control_timer_callback, this));

  RCLCPP_INFO(this->get_logger(), "Nodo de teleoperación por bumper iniciado");
}

void BumperLoggerNode::bumper_callback(const kobuki_ros_interfaces::msg::BumperEvent::SharedPtr msg){
  bool pressed = (msg->state == kobuki_ros_interfaces::msg::BumperEvent::PRESSED);  // Ajustar según la definición real

  // Actualizar estado según el bumper
  if (msg->bumper == kobuki_ros_interfaces::msg::BumperEvent::LEFT) {  // izquierdo
    if (left_pressed_ != pressed) {
      left_pressed_ = pressed;
      RCLCPP_INFO(this->get_logger(), "Bumper izquierdo %s", pressed ? "PULSADO" : "LIBRE");
    }
  } else if (msg->bumper == kobuki_ros_interfaces::msg::BumperEvent::CENTER) {  // central
    if (center_pressed_ != pressed) {
      center_pressed_ = pressed;
      RCLCPP_INFO(this->get_logger(), "Bumper central %s", pressed ? "PULSADO" : "LIBRE");
    }
  } else if (msg->bumper == kobuki_ros_interfaces::msg::BumperEvent::RIGHT) {  // derecho
    if (right_pressed_ != pressed) {
      right_pressed_ = pressed;
      RCLCPP_INFO(this->get_logger(), "Bumper derecho %s", pressed ? "PULSADO" : "LIBRE");
    }
  }
}

void BumperLoggerNode::control_timer_callback()
{
  RCLCPP_DEBUG(this->get_logger(), "Estado: L=%d C=%d R=%d", 
               left_pressed_, center_pressed_, right_pressed_);
}