// src/say_text_action.cpp

#include "waiter_robot/say_text_action.hpp"
#include "waiter_robot/text_utils.hpp"

SayTextAction::SayTextAction(
  const std::string & name,
  const BT::NodeConfiguration & config,
  rclcpp::Node::SharedPtr node)
: BT::SyncActionNode(name, config), node_(node)
{
}

BT::PortsList SayTextAction::providedPorts()
{
  return {BT::InputPort<std::string>("text", "Text to speak (supports {var} placeholders)")};
}

BT::NodeStatus SayTextAction::tick()
{
  std::string raw_text;
  if (!getInput("text", raw_text)) {
    RCLCPP_ERROR(node_->get_logger(), "SayText: missing required port 'text'");
    return BT::NodeStatus::FAILURE;
  }

  // FIX: expandir variables del blackboard, p.ej. {bebida} -> "cafe"
  std::string text = waiter_robot::formatText(raw_text, config().blackboard);

  RCLCPP_INFO(node_->get_logger(), "[SayText] Robot dice: '%s'", text.c_str());
  return BT::NodeStatus::SUCCESS;
}