#include "waiter_robot/listen_text_action.hpp"

ListenTextAction::ListenTextAction(
    const std::string& name,
    const BT::NodeConfiguration& config,
    rclcpp::Node::SharedPtr node)
  : BT::SyncActionNode(name, config), node_(node)
{
}

BT::PortsList ListenTextAction::providedPorts() {
  return { BT::OutputPort<std::string>("recognized_text", "Simulated order") };
}

BT::NodeStatus ListenTextAction::tick() {
  // Simular que el cliente pide "café" y "tostada"
  std::string recognized = "Quiero un café y una tostada";
  
  RCLCPP_INFO(node_->get_logger(), "👂 Robot heard: '%s'", recognized.c_str());
  setOutput("recognized_text", recognized);
  
  return BT::NodeStatus::SUCCESS;
}