#ifndef WAITER_ROBOT__EXTRACT_INFO_CLIENT_ACTION_HPP_
#define WAITER_ROBOT__EXTRACT_INFO_CLIENT_ACTION_HPP_

#include <behaviortree_cpp/action_node.h>
#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <string>
#include <chrono>

class HRIClient;

class ExtractInfoClientAction : public BT::StatefulActionNode
{
public:
  ExtractInfoClientAction(const std::string& name,
                          const BT::NodeConfiguration& config,
                          std::shared_ptr<HRIClient> hri_client);
  
  static BT::PortsList providedPorts();
  
  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  std::shared_ptr<HRIClient> hri_client_;
  std::chrono::steady_clock::time_point start_time_;
};

#endif  // WAITER_ROBOT__EXTRACT_INFO_CLIENT_ACTION_HPP_