#include "waiter_robot/say_text_action.hpp"

SayTextAction::SayTextAction(
    const std::string& name,
    const BT::NodeConfiguration& config,
    rclcpp::Node::SharedPtr node)
  : BT::SyncActionNode(name, config), node_(node)
{
}

BT::PortsList SayTextAction::providedPorts() {
  return { BT::InputPort<std::string>("text", "Text to speak") };
}

BT::NodeStatus SayTextAction::tick() {
  std::string text;
  if (!getInput("text", text)) {
    return BT::NodeStatus::FAILURE;
  }
  
  RCLCPP_INFO(node_->get_logger(), "🤖 Robot says: '%s'", text.c_str());
  return BT::NodeStatus::SUCCESS;
}