#ifndef PATROL_FSM__PATROL_STATES_HPP_
#define PATROL_FSM__PATROL_STATES_HPP_

#include "rclcpp/rclcpp.hpp"
#include <memory>
#include <string>

namespace patrol_fsm
{

class PatrolFSMNode;

class PatrolState
{
public:
  virtual void on_entry() = 0;
  virtual void on_do() = 0;
  virtual PatrolState* check_transitions() = 0;
  virtual void on_exit() = 0;
  virtual ~PatrolState() = default;
  virtual std::string get_name() const = 0;
};

class InitState : public PatrolState
{
  PatrolFSMNode* node_;
public:
  explicit InitState(PatrolFSMNode* node);
  void on_entry() override;
  void on_do() override;
  PatrolState* check_transitions() override;
  void on_exit() override;
  std::string get_name() const override { return "INIT"; }
};

class NavigatingState : public PatrolState
{
  PatrolFSMNode* node_;
  bool goal_sent_ = false;
public:
  explicit NavigatingState(PatrolFSMNode* node);
  void on_entry() override;
  void on_do() override;
  PatrolState* check_transitions() override;
  void on_exit() override;
  std::string get_name() const override { return "NAVIGATING"; }
};

class RecoveryState : public PatrolState
{
  PatrolFSMNode* node_;
  rclcpp::Time entry_time_;
  rclcpp::Duration wait_time_{3, 0};
public:
  explicit RecoveryState(PatrolFSMNode* node);
  void on_entry() override;
  void on_do() override;
  PatrolState* check_transitions() override;
  void on_exit() override;
  std::string get_name() const override { return "RECOVERY"; }
};

class StateMachine
{
  PatrolState* current_state_;
  rclcpp::Logger logger_;
public:
  StateMachine(PatrolState* initial_state, rclcpp::Logger logger);
  ~StateMachine();
  void step();
  std::string get_current_state_name() const;
};

}  // namespace patrol_fsm
#endif