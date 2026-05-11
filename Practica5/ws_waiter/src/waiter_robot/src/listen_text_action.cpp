// src/listen_text_action.cpp
#include "waiter_robot/listen_text_action.hpp"

ListenTextAction::ListenTextAction(
  const std::string & name,
  const BT::NodeConfiguration & config,
  rclcpp::Node::SharedPtr node)
: BT::SyncActionNode(name, config), node_(node)
{
}

BT::PortsList ListenTextAction::providedPorts()
{
  return {BT::OutputPort<std::string>("recognized_text", "Simulated speech recognition result")};
}

BT::NodeStatus ListenTextAction::tick()
{
  // Simulacion: el cliente siempre pide "cafe" y "tostada"
  const std::string recognized = "Quiero un cafe y una tostada";

  RCLCPP_INFO(node_->get_logger(), "[ListenText] Escuchado: '%s'", recognized.c_str());
  setOutput("recognized_text", recognized);

  return BT::NodeStatus::SUCCESS;
}