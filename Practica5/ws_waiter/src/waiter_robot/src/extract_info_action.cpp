#include "waiter_robot/extract_info_action.hpp"

ExtractInfoAction::ExtractInfoAction(
    const std::string& name,
    const BT::NodeConfiguration& config,
    rclcpp::Node::SharedPtr node)
  : BT::SyncActionNode(name, config), node_(node)
{
}

BT::PortsList ExtractInfoAction::providedPorts() {
  return {
    BT::InputPort<std::string>("interest", "What to extract"),
    BT::InputPort<std::string>("full_text", "Full text to analyze"),
    BT::OutputPort<std::string>("extracted_info", "Extracted information")
  };
}

BT::NodeStatus ExtractInfoAction::tick() {
  std::string interest, full_text;
  if (!getInput("interest", interest) || !getInput("full_text", full_text)) {
    return BT::NodeStatus::FAILURE;
  }
  
  std::string extracted;
  if (interest == "bebida") {
    extracted = "café";
  } else if (interest == "comida") {
    extracted = "tostada";
  } else {
    extracted = "desconocido";
  }
  
  RCLCPP_INFO(node_->get_logger(), "📋 Extracted '%s': '%s'", 
              interest.c_str(), extracted.c_str());
  setOutput("extracted_info", extracted);
  
  return BT::NodeStatus::SUCCESS;
}