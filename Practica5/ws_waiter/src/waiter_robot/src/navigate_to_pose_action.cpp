// src/navigate_to_pose_action.cpp

#include "waiter_robot/navigate_to_pose_action.hpp"
#include <tf2/LinearMath/Quaternion.h>

NavigateToPoseAction::NavigateToPoseAction(
  const std::string & name,
  const BT::NodeConfiguration & config,
  rclcpp::Node::SharedPtr node)
: BT::StatefulActionNode(name, config), node_(node)
{
  nav_client_ = rclcpp_action::create_client<NavigateToPose>(node_, "navigate_to_pose");
}

BT::PortsList NavigateToPoseAction::providedPorts()
{
  return {
    BT::InputPort<double>("x",   0.0, "Target X (m)"),
    BT::InputPort<double>("y",   0.0, "Target Y (m)"),
    BT::InputPort<double>("yaw", 0.0, "Target yaw (rad)")
  };
}

geometry_msgs::msg::PoseStamped NavigateToPoseAction::create_pose(
  double x, double y, double yaw)
{
  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = "map";
  pose.header.stamp    = node_->now();
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

BT::NodeStatus NavigateToPoseAction::onStart()
{
  double x, y, yaw;
  if (!getInput("x", x) || !getInput("y", y) || !getInput("yaw", yaw)) {
    RCLCPP_ERROR(node_->get_logger(), "NavigateToPose: faltan puertos de entrada");
    return BT::NodeStatus::FAILURE;
  }

  RCLCPP_INFO(node_->get_logger(),
    "[NavigateToPose] Navegando a (%.2f, %.2f, yaw=%.2f)", x, y, yaw);

  if (!nav_client_->wait_for_action_server(std::chrono::seconds(5))) {
    RCLCPP_ERROR(node_->get_logger(),
      "NavigateToPose: servidor Nav2 no disponible (timeout 5 s)");
    return BT::NodeStatus::FAILURE;
  }

  goal_sent_    = false;
  goal_done_    = false;
  goal_success_ = false;

  auto goal_msg  = NavigateToPose::Goal();
  goal_msg.pose  = create_pose(x, y, yaw);

  auto opts = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();

  // FIX: firma correcta para ROS2 Jazzy
  opts.goal_response_callback =
    [this](const GoalHandleNav::SharedPtr & handle) {
      if (!handle) {
        RCLCPP_ERROR(node_->get_logger(), "NavigateToPose: goal rechazado por Nav2");
        goal_done_    = true;
        goal_success_ = false;
      } else {
        RCLCPP_INFO(node_->get_logger(), "NavigateToPose: goal aceptado por Nav2");
        goal_handle_ = handle;
      }
    };

  opts.result_callback =
    [this](const GoalHandleNav::WrappedResult & result) {
      goal_done_ = true;
      if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
        RCLCPP_INFO(node_->get_logger(), "NavigateToPose: exito");
        goal_success_ = true;
      } else {
        RCLCPP_ERROR(node_->get_logger(),
          "NavigateToPose: fallo (codigo %d)", static_cast<int>(result.code));
        goal_success_ = false;
      }
    };

  nav_client_->async_send_goal(goal_msg, opts);
  goal_sent_ = true;

  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus NavigateToPoseAction::onRunning()
{
  if (!goal_sent_) {
    return BT::NodeStatus::RUNNING;
  }
  if (goal_done_) {
    return goal_success_ ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }
  return BT::NodeStatus::RUNNING;
}

void NavigateToPoseAction::onHalted()
{
  if (goal_handle_) {
    RCLCPP_INFO(node_->get_logger(), "NavigateToPose: cancelando navegacion");
    nav_client_->async_cancel_goal(goal_handle_);
  }
  goal_done_    = true;
  goal_success_ = false;
}