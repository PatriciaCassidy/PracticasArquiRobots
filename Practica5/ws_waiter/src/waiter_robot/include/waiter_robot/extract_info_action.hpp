// include/waiter_robot/extract_info_action.hpp
#ifndef WAITER_ROBOT__EXTRACT_INFO_ACTION_HPP_
#define WAITER_ROBOT__EXTRACT_INFO_ACTION_HPP_

#include <behaviortree_cpp/action_node.h>
#include <rclcpp/rclcpp.hpp>
#include <string>

class ExtractInfoAction : public BT::SyncActionNode
{
public:
  ExtractInfoAction(const std::string & name,
                    const BT::NodeConfiguration & config,
                    rclcpp::Node::SharedPtr node);

  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;

private:
  rclcpp::Node::SharedPtr node_;
};

#endif  // WAITER_ROBOT__EXTRACT_INFO_ACTION_HPP_