#ifndef WAITER_ROBOT__NAVIGATE_TO_POSE_ACTION_HPP_
#define WAITER_ROBOT__NAVIGATE_TO_POSE_ACTION_HPP_

#include <behaviortree_cpp/action_node.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <memory>
#include <string>

class NavigateToPoseAction : public BT::StatefulActionNode
{
public:
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using GoalHandleNav = rclcpp_action::ClientGoalHandle<NavigateToPose>;

  NavigateToPoseAction(const std::string& name,
                       const BT::NodeConfiguration& config,
                       rclcpp::Node::SharedPtr node);
  
  static BT::PortsList providedPorts();
  
  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp_action::Client<NavigateToPose>::SharedPtr nav_client_;
  std::shared_ptr<GoalHandleNav> goal_handle_;
  
  bool goal_sent_ = false;
  bool goal_success_ = false;
  bool goal_done_ = false;
  
  geometry_msgs::msg::PoseStamped create_pose(double x, double y, double yaw);
};

#endif  // WAITER_ROBOT__NAVIGATE_TO_POSE_ACTION_HPP_