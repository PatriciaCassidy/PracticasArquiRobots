// Copyright 2024 Intelligent Robotics Lab
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

#ifndef LASER__NEARESTOBSTACLENODE_HPP_
#define LASER__NEARESTOBSTACLENODE_HPP_

#include <memory>
#include <string> 

#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/transform_broadcaster.h"

#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"

namespace laser
{

class NearestObstacleNode : public rclcpp::Node
{
public:
  NearestObstacleNode();

protected:
  void laser_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr & scan);
  bool is_obstacle(const sensor_msgs::msg::LaserScan & scan, float & distance, float angle, size_t & index)const;
  bool transform_to_robot_frame(const geometry_msgs::msg::PointStamped & sensor_point, geometry_msgs::msg::PointStamped & robot_point);
  void print_obstacle_info(const sensor_msgs::msg::LaserScan & scan, float distance, float angle, size_t index, const geometry_msgs::msg::PointStamped & robot_point) const;

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr nearest_obstacle_pub_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_; 
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  std::string robot_frame_;
  std::string obstacle_frame_;
  float range_min_threshold_;
  float range_max_threshold_;
  bool enable_debug_output_;
};

}  // namespace laser

#endif  // LASER__NEARESTOBSTACLENODE_HPP_
