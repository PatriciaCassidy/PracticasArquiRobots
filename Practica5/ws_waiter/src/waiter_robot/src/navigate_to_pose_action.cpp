#include "waiter_robot/navigate_to_pose_action.hpp"
#include <tf2/LinearMath/Quaternion.h>

NavigateToPoseAction::NavigateToPoseAction(
    const std::string& name,
    const BT::NodeConfiguration& config,
    rclcpp::Node::SharedPtr node)
  : BT::StatefulActionNode(name, config), node_(node)
{
  nav_client_ = rclcpp_action::create_client<NavigateToPose>(
    node_, "navigate_to_pose");
}

BT::PortsList NavigateToPoseAction::providedPorts() {
  return {
    BT::InputPort<double>("x", 0.0, "Target X coordinate"),
    BT::InputPort<double>("y", 0.0, "Target Y coordinate"),
    BT::InputPort<double>("yaw", 0.0, "Target yaw angle")
  };
}

geometry_msgs::msg::PoseStamped NavigateToPoseAction::create_pose(
    double x, double y, double yaw)
{
  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = "map";
  pose.header.stamp = node_->now();
  pose.pose.position.x = x;
  pose.pose.position.y = y;
  pose.pose.position.z = 0.0;
  
  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, yaw);
  pose.pose.orientation.x = q.x();
  pose.pose.orientation.y = q.y();
  pose.pose.orientation.z = q.z();
  pose.pose.orientation.w = q.w();
  
  return pose;
}

BT::NodeStatus NavigateToPoseAction::onStart() {
  double x, y, yaw;
  if (!getInput("x", x) || !getInput("y", y) || !getInput("yaw", yaw)) {
    RCLCPP_ERROR(node_->get_logger(), "Missing required input ports");
    return BT::NodeStatus::FAILURE;
  }
  
  RCLCPP_INFO(node_->get_logger(), "Navigating to (%.2f, %.2f, %.2f)", x, y, yaw);
  
  auto goal_msg = NavigateToPose::Goal();
  goal_msg.pose = create_pose(x, y, yaw);
  
  auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
  send_goal_options.goal_response_callback =
    [this](const GoalHandleNav::SharedPtr& goal_handle) {
      if (!goal_handle) {
        RCLCPP_ERROR(node_->get_logger(), "Goal rejected by Nav2");
        goal_done_ = true;
        goal_success_ = false;
      } else {
        RCLCPP_INFO(node_->get_logger(), "Goal accepted by Nav2");
        goal_handle_ = goal_handle;
      }
    };
    
  send_goal_options.result_callback =
    [this](const GoalHandleNav::WrappedResult& result) {
      goal_done_ = true;
      switch (result.code) {
        case rclcpp_action::ResultCode::SUCCEEDED:
          RCLCPP_INFO(node_->get_logger(), "Navigation succeeded");
          goal_success_ = true;
          break;
        default:
          RCLCPP_ERROR(node_->get_logger(), "Navigation failed with code: %d", 
                       static_cast<int>(result.code));
          goal_success_ = false;
          break;
      }
    };
  
  goal_sent_ = false;
  goal_done_ = false;
  goal_success_ = false;
  
  if (!nav_client_->wait_for_action_server(std::chrono::seconds(5))) {
    RCLCPP_ERROR(node_->get_logger(), "Nav2 action server not available");
    return BT::NodeStatus::FAILURE;
  }
  
  nav_client_->async_send_goal(goal_msg, send_goal_options);
  goal_sent_ = true;
  
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus NavigateToPoseAction::onRunning() {
  if (!goal_sent_) {
    return BT::NodeStatus::RUNNING;
  }
  
  if (goal_done_) {
    return goal_success_ ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }
  
  return BT::NodeStatus::RUNNING;
}

void NavigateToPoseAction::onHalted() {
  if (goal_handle_) {
    RCLCPP_INFO(node_->get_logger(), "Cancelling navigation");
    nav_client_->async_cancel_goal(goal_handle_);
  }
}