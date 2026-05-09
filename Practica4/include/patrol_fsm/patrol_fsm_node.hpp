#ifndef PATROL_FSM__PATROL_FSM_NODE_HPP_
#define PATROL_FSM__PATROL_FSM_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_example/navigation_client.hpp"
#include "patrol_fsm/patrol_states.hpp"
#include <memory>
#include <vector>
#include <string>
#include <sstream>

namespace patrol_fsm
{

class PatrolFSMNode : public rclcpp::Node
{
public:
  PatrolFSMNode();

  bool is_server_ready() const;
  bool is_goal_done() const;
  bool is_goal_successful() const;
  
  void send_current_waypoint();
  void advance_to_next_waypoint();

  int get_current_waypoint_index() const { return current_waypoint_index_; }
  int get_max_retries() const { return max_retries_; }
  int get_retry_count() const { return retry_count_; }
  void increment_retry_count() { retry_count_++; }
  void reset_retry_count() { retry_count_ = 0; }
  
  double get_goal_timeout() const { return goal_timeout_; }
  void publish_state(const std::string& state_name);

  std::shared_ptr<NavigationClient> nav_client_;
  rclcpp::Time goal_start_time_;
  bool goal_started_ = false;

private:
  void control_cycle();
  void load_waypoints();
  void declare_ros_parameters();

  StateMachine* fsm_;
  std::vector<geometry_msgs::msg::PoseStamped> waypoints_;
  int current_waypoint_index_ = 0;
  
  int max_retries_ = 3;
  int retry_count_ = 0;
  double goal_timeout_ = 120.0;
  
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  friend class InitState;
  friend class NavigatingState;
  friend class RecoveryState;
};

}  // namespace patrol_fsm
#endif