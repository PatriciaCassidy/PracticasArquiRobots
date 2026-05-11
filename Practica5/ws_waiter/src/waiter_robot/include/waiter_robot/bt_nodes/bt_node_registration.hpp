#ifndef WAITER_ROBOT__BT_NODES__BT_NODE_REGISTRATION_HPP_
#define WAITER_ROBOT__BT_NODES__BT_NODE_REGISTRATION_HPP_

#include <behaviortree_cpp/bt_factory.h>
#include <rclcpp/rclcpp.hpp>
#include <memory>

void register_waiter_nodes(
  BT::BehaviorTreeFactory& factory,
  rclcpp::Node::SharedPtr node);

#endif  // WAITER_ROBOT__BT_NODES__BT_NODE_REGISTRATION_HPP_