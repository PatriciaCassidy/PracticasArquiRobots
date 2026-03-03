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

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <string>

#include "laser/NearestObstacleNode.hpp"

#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

#include "rclcpp/rclcpp.hpp"

#include "tf2/exceptions.h"
#include "tf2/time.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace laser
{

using std::placeholders::_1;

NearestObstacleNode::NearestObstacleNode()
: Node("nearest_obstacle_node"),
  tf_buffer_(this->get_clock()),
  tf_listener_(tf_buffer_)
{
  declare_parameter("robot_frame", "base_link");
  declare_parameter("obstacle_frame", "nearest_obstacle");
  declare_parameter("range_min_threshold", 0.0);
  declare_parameter("range_max_threshold", std::numeric_limits<double>::max());
  declare_parameter("enable_debug_output", true);

  get_parameter("robot_frame", robot_frame_);
  get_parameter("obstacle_frame", obstacle_frame_);
  get_parameter("range_min_threshold", range_min_threshold_);
  get_parameter("range_max_threshold", range_max_threshold_);
  get_parameter("enable_debug_output", enable_debug_output_);

  RCLCPP_INFO(get_logger(), "NearestObstacleNode initialized");
  RCLCPP_INFO(get_logger(), "  robot_frame: %s", robot_frame_.c_str());
  RCLCPP_INFO(get_logger(), "  obstacle_frame: %s", obstacle_frame_.c_str());
  RCLCPP_INFO(get_logger(), "  range_min_threshold: %f", range_min_threshold_);
  RCLCPP_INFO(get_logger(), "  range_max_threshold: %f", range_max_threshold_);
  RCLCPP_INFO(get_logger(), "  enable_debug_output: %s", enable_debug_output_ ? "true" : "false");

  laser_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
    "input_scan", 
    rclcpp::SensorDataQoS().reliable(),
    std::bind(&NearestObstacleNode::laser_callback, this, _1));

  // Create publisher for PointStamped
  nearest_obstacle_pub_ = create_publisher<geometry_msgs::msg::PointStamped>(
    "nearest_obstacle", 10);

  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(*this);
}

find_closest_valid_point(
  const sensor_msgs::msg::LaserScan & scan,
  float & distance,
  float & angle,
  size_t & index) const
{
  distance = std::numeric_limits<float>::infinity();
  bool found_valid = false;

  for (size_t i = 0; i < scan.ranges.size(); ++i) {
    const float range = scan.ranges[i];
    
    // Filter invalid ranges (NaN, Inf)
    if (!std::isfinite(range)) {
      continue;
    }
    
    // Filter ranges outside sensor limits
    if (range < scan.range_min || range > scan.range_max) {
      continue;
    }
    
    // Apply user thresholds if specified
    if (range < range_min_threshold_ || range > range_max_threshold_) {
      continue;
    }

    if (range < distance) {
      distance = range;
      index = i;
      found_valid = true;
    }
  }

  if (found_valid) {
    angle = scan.angle_min + static_cast<float>(index) * scan.angle_increment;
  }

  return found_valid;
}

bool
NearestObstacleNode::transform_to_robot_frame(
  const geometry_msgs::msg::PointStamped & sensor_point,
  geometry_msgs::msg::PointStamped & robot_point)
{
  try {
    // Lookup transform from sensor frame to robot frame at the exact timestamp
    auto transform = tf_buffer_.lookupTransform(
      robot_frame_,
      sensor_point.header.frame_id,
      sensor_point.header.stamp,
      tf2::durationFromSec(0.1));  // 100ms timeout

    // Apply transform
    tf2::doTransform(sensor_point, robot_point, transform);
    return true;
    
  } catch (const tf2::TransformException & ex) {
    RCLCPP_DEBUG(get_logger(), "Could not transform point to %s: %s", 
                 robot_frame_.c_str(), ex.what());
    return false;
  }
}

void
NearestObstacleNode::print_obstacle_info(
  const sensor_msgs::msg::LaserScan & scan,
  float distance,
  float angle,
  size_t index,
  const geometry_msgs::msg::PointStamped & robot_point) const
{
  if (!enable_debug_output_) {
    return;
  }

  RCLCPP_INFO(get_logger(), "=== Nearest Obstacle Detected ===");
  RCLCPP_INFO(get_logger(), "  LaserScan frame: %s", scan.header.frame_id.c_str());
  RCLCPP_INFO(get_logger(), "  Timestamp: %d.%d", 
              scan.header.stamp.sec, scan.header.stamp.nanosec);
  RCLCPP_INFO(get_logger(), "  Index in scan: %zu of %zu", index, scan.ranges.size());
  RCLCPP_INFO(get_logger(), "  Raw distance: %.3f m", distance);
  RCLCPP_INFO(get_logger(), "  Raw angle: %.3f rad (%.2f deg)", angle, angle * 180.0 / M_PI);
  
  // Position in sensor frame
  float sensor_x = distance * std::cos(angle);
  float sensor_y = distance * std::sin(angle);
  RCLCPP_INFO(get_logger(), "  Position in %s: (%.3f, %.3f, 0.0)", 
              scan.header.frame_id.c_str(), sensor_x, sensor_y);
  
  // Position in robot frame (after TF)
  RCLCPP_INFO(get_logger(), "  Position in %s: (%.3f, %.3f, %.3f)", 
              robot_frame_.c_str(),
              robot_point.point.x, robot_point.point.y, robot_point.point.z);
  
  // Additional info about the laser scan
  RCLCPP_INFO(get_logger(), "  LaserScan angles: min=%.2f deg, max=%.2f deg, increment=%.3f deg",
              scan.angle_min * 180.0 / M_PI,
              scan.angle_max * 180.0 / M_PI,
              scan.angle_increment * 180.0 / M_PI);
  
  RCLCPP_INFO(get_logger(), "================================");

}

void
NearestObstacleNode::laser_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr & scan_msg)
{
  // Check if scan is empty
  if (scan_msg->ranges.empty()) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, 
                         "Received empty LaserScan");
    return;
  }

  // Find closest valid point
  float min_distance, min_angle;
  size_t min_index;
  
  if (!find_closest_valid_point(*scan_msg, min_distance, min_angle, min_index)) {
    RCLCPP_DEBUG(get_logger(), "No valid points in LaserScan");
    return;
  }

  // Calculate position in sensor frame
  float obstacle_x = min_distance * std::cos(min_angle);
  float obstacle_y = min_distance * std::sin(min_angle);

  // Create point in sensor frame
  geometry_msgs::msg::PointStamped sensor_point;
  sensor_point.header = scan_msg->header;  // Use same timestamp and frame_id as laser scan
  sensor_point.point.x = obstacle_x;
  sensor_point.point.y = obstacle_y;
  sensor_point.point.z = 0.0;

  RCLCPP_DEBUG(get_logger(), 
               "Closest obstacle: distance=%.3f m, angle=%.3f rad, pos in %s=(%.3f, %.3f)",
               min_distance, min_angle, sensor_point.header.frame_id.c_str(), 
               obstacle_x, obstacle_y);

  // Transform to robot frame
  geometry_msgs::msg::PointStamped robot_point;
  if (!transform_to_robot_frame(sensor_point, robot_point)) {
    RCLCPP_DEBUG(get_logger(), "Skipping publication due to TF failure");
    return;
  }

  // Print detailed info (if enabled)
  print_obstacle_info(*scan_msg, min_distance, min_angle, min_index, robot_point);

  // Publish PointStamped in robot frame
  robot_point.header.frame_id = robot_frame_;  // Ensure correct frame
  nearest_obstacle_pub_->publish(robot_point);
  
  RCLCPP_DEBUG(get_logger(), "Published nearest_obstacle in %s frame at (%.3f, %.3f, %.3f)", 
               robot_frame_.c_str(),
               robot_point.point.x, robot_point.point.y, robot_point.point.z);

  // Create and publish TF for the obstacle
  geometry_msgs::msg::TransformStamped obstacle_transform;
  obstacle_transform.header.stamp = scan_msg->header.stamp;  // Coherent timestamp
  obstacle_transform.header.frame_id = robot_frame_;         // Parent frame (robot)
  obstacle_transform.child_frame_id = obstacle_frame_;       // Child frame (obstacle)
  
  // Set translation (position of obstacle relative to robot)
  obstacle_transform.transform.translation.x = robot_point.point.x;
  obstacle_transform.transform.translation.y = robot_point.point.y;
  obstacle_transform.transform.translation.z = robot_point.point.z;
  
  // Set rotation (identity - no rotation for the obstacle frame)
  obstacle_transform.transform.rotation.x = 0.0;
  obstacle_transform.transform.rotation.y = 0.0;
  obstacle_transform.transform.rotation.z = 0.0;
  obstacle_transform.transform.rotation.w = 1.0;

  // Broadcast the transform
  tf_broadcaster_->sendTransform(obstacle_transform);
  RCLCPP_DEBUG(get_logger(), "Broadcasted TF: %s -> %s", 
               robot_frame_.c_str(), obstacle_frame_.c_str());

}



}  // namespace laser
