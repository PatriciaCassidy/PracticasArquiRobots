// include/waiter_robot/listen_text_action.hpp
#ifndef WAITER_ROBOT__LISTEN_TEXT_ACTION_HPP_
#define WAITER_ROBOT__LISTEN_TEXT_ACTION_HPP_

#include <behaviortree_cpp/action_node.h>
#include <rclcpp/rclcpp.hpp>
#include <string>

class ListenTextAction : public BT::SyncActionNode
{
public:
  ListenTextAction(const std::string & name,
                   const BT::NodeConfiguration & config,
                   rclcpp::Node::SharedPtr node);

  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;

private:
  rclcpp::Node::SharedPtr node_;
};

#endif  // WAITER_ROBOT__LISTEN_TEXT_ACTION_HPP_