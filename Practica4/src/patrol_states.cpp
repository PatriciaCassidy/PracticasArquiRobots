#include "patrol_fsm/patrol_states.hpp"
#include "patrol_fsm/patrol_fsm_node.hpp"

namespace patrol_fsm
{

StateMachine::StateMachine(PatrolState* initial_state, rclcpp::Logger logger)
  : current_state_(initial_state), logger_(logger)
{
  RCLCPP_INFO(logger_, "FSM iniciada en: %s", current_state_->get_name().c_str());
  current_state_->on_entry();
}

StateMachine::~StateMachine() { delete current_state_; }

void StateMachine::step()
{
  current_state_->on_do();
  PatrolState* next_state = current_state_->check_transitions();
  
  if (next_state != nullptr) {
    RCLCPP_INFO(logger_, "Transicion: %s -> %s",
                current_state_->get_name().c_str(), next_state->get_name().c_str());
    current_state_->on_exit();
    delete current_state_;
    current_state_ = next_state;
    current_state_->on_entry();
  }
}

std::string StateMachine::get_current_state_name() const
{
  return current_state_->get_name();
}

// InitState
InitState::InitState(PatrolFSMNode* node) : node_(node) {}

void InitState::on_entry()
{
  RCLCPP_INFO(node_->get_logger(), "[INIT] Esperando servidor Nav2...");
  node_->publish_state("INIT");
}

void InitState::on_do() {}

PatrolState* InitState::check_transitions()
{
  if (node_->is_server_ready()) {
    RCLCPP_INFO(node_->get_logger(), "[INIT] Nav2 detectado!");
    return new NavigatingState(node_);
  }
  return nullptr;
}

void InitState::on_exit()
{
  RCLCPP_INFO(node_->get_logger(), "[INIT] Saliendo...");
}

// NavigatingState
NavigatingState::NavigatingState(PatrolFSMNode* node) : node_(node), goal_sent_(false) {}

void NavigatingState::on_entry()
{
  RCLCPP_INFO(node_->get_logger(), "[NAVIGATING] Waypoint %d", 
              node_->get_current_waypoint_index());
  node_->publish_state("NAVIGATING");
  goal_sent_ = false;
}

void NavigatingState::on_do()
{
  if (!goal_sent_) {
    node_->send_current_waypoint();
    goal_sent_ = true;
    node_->goal_start_time_ = node_->now();
    node_->goal_started_ = true;
  }
}

PatrolState* NavigatingState::check_transitions()
{
  if (node_->goal_started_) {
    auto elapsed = node_->now() - node_->goal_start_time_;
    if (elapsed.seconds() > node_->get_goal_timeout()) {
      RCLCPP_WARN(node_->get_logger(), "[NAVIGATING] Timeout!");
      node_->nav_client_->cancel_goal();
      node_->goal_started_ = false;
      return new RecoveryState(node_);
    }
  }
  
  if (goal_sent_ && node_->is_goal_done()) {
    node_->goal_started_ = false;
    if (node_->is_goal_successful()) {
      RCLCPP_INFO(node_->get_logger(), "[NAVIGATING] EXITO!");
      node_->reset_retry_count();
      node_->advance_to_next_waypoint();
      return new NavigatingState(node_);
    } else {
      RCLCPP_WARN(node_->get_logger(), "[NAVIGATING] Fallo!");
      return new RecoveryState(node_);
    }
  }
  return nullptr;
}

void NavigatingState::on_exit()
{
  RCLCPP_INFO(node_->get_logger(), "[NAVIGATING] Saliendo...");
}

// RecoveryState
RecoveryState::RecoveryState(PatrolFSMNode* node) : node_(node) {}

void RecoveryState::on_entry()
{
  entry_time_ = node_->now();
  node_->increment_retry_count();
  RCLCPP_WARN(node_->get_logger(), "[RECOVERY] Intento %d/%d", 
              node_->get_retry_count(), node_->get_max_retries());
  node_->publish_state("RECOVERY");
}

void RecoveryState::on_do() {}

PatrolState* RecoveryState::check_transitions()
{
  auto elapsed = node_->now() - entry_time_;
  if (elapsed < wait_time_) return nullptr;
  
  if (node_->get_retry_count() < node_->get_max_retries()) {
    return new NavigatingState(node_);
  }
  
  RCLCPP_WARN(node_->get_logger(), "[RECOVERY] Saltando waypoint");
  node_->reset_retry_count();
  node_->advance_to_next_waypoint();
  return new NavigatingState(node_);
}

void RecoveryState::on_exit()
{
  RCLCPP_INFO(node_->get_logger(), "[RECOVERY] Saliendo...");
}

}  // namespace patrol_fsm