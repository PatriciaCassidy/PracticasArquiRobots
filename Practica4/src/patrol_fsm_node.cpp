#include "patrol_fsm/patrol_fsm_node.hpp"

namespace patrol_fsm
{

PatrolFSMNode::PatrolFSMNode()
: Node("patrol_fsm_node")
{
  declare_ros_parameters();
  
  nav_client_ = std::make_shared<NavigationClient>();
  state_pub_ = create_publisher<std_msgs::msg::String>("/patrol_state", 10);
  
  load_waypoints();
  
  fsm_ = new StateMachine(new InitState(this), get_logger());
  
  timer_ = create_wall_timer(
    std::chrono::milliseconds(500),
    std::bind(&PatrolFSMNode::control_cycle, this));
  
  RCLCPP_INFO(get_logger(), "FSM patrullaje lista. %zu waypoints.", waypoints_.size());
}

void PatrolFSMNode::declare_ros_parameters()
{
  declare_parameter<int>("max_retries", 3);
  declare_parameter<double>("goal_timeout", 120.0);
  get_parameter("max_retries", max_retries_);
  get_parameter("goal_timeout", goal_timeout_);
}

void PatrolFSMNode::load_waypoints()
{
  waypoints_.push_back(nav_client_->create_pose_stamped(2.0, 2.0, 0.0));
  waypoints_.push_back(nav_client_->create_pose_stamped(4.0, 3.0, 1.57));
  waypoints_.push_back(nav_client_->create_pose_stamped(6.0, 1.0, 3.14));
  waypoints_.push_back(nav_client_->create_pose_stamped(3.0, -1.0, -1.57));
  waypoints_.push_back(nav_client_->create_pose_stamped(0.0, 0.0, 0.0));
}

void PatrolFSMNode::control_cycle() { fsm_->step(); }

bool PatrolFSMNode::is_server_ready() const
{
  return nav_client_->wait_for_action_server(std::chrono::seconds(1));
}

bool PatrolFSMNode::is_goal_done() const { return nav_client_->is_goal_done(); }
bool PatrolFSMNode::is_goal_successful() const { return nav_client_->was_goal_successful(); }

void PatrolFSMNode::send_current_waypoint()
{
  if (current_waypoint_index_ < static_cast<int>(waypoints_.size())) {
    RCLCPP_INFO(get_logger(), "Enviando WP %d/%zu", 
                current_waypoint_index_ + 1, waypoints_.size());
    nav_client_->send_goal(waypoints_[current_waypoint_index_]);
  }
}

void PatrolFSMNode::advance_to_next_waypoint()
{
  current_waypoint_index_++;
  if (current_waypoint_index_ >= static_cast<int>(waypoints_.size())) {
    RCLCPP_INFO(get_logger(), "CICLO COMPLETO! Reiniciando.");
    current_waypoint_index_ = 0;
  }
}

void PatrolFSMNode::publish_state(const std::string& state_name)
{
  auto msg = std_msgs::msg::String();
  std::stringstream ss;
  ss << state_name << " | WP:" << (current_waypoint_index_ + 1) 
     << "/" << waypoints_.size()
     << " | Retry:" << retry_count_ << "/" << max_retries_;
  msg.data = ss.str();
  state_pub_->publish(msg);
}

}  // namespace patrol_fsm