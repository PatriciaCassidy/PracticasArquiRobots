#include "waiter_robot/say_text_client_action.hpp"
#include "waiter_robot/text_utils.hpp"
#include "hri_example/hri_client.hpp"

SayTextClientAction::SayTextClientAction(
    const std::string& name,
    const BT::NodeConfiguration& config,
    std::shared_ptr<HRIClient> hri_client)
  : BT::StatefulActionNode(name, config), hri_client_(hri_client)
{
}

BT::PortsList SayTextClientAction::providedPorts() {
  return { BT::InputPort<std::string>("text", "Text to speak") };
}

std::string SayTextClientAction::formatText(const std::string& text) {
  return waiter_robot::formatText(text, config().blackboard);
}

BT::NodeStatus SayTextClientAction::onStart() {
  std::string text;
  if (!getInput("text", text)) {
    RCLCPP_ERROR(rclcpp::get_logger("SayTextClient"), 
                 "Missing 'text' input port");
    return BT::NodeStatus::FAILURE;
  }
  
  text = formatText(text);
  RCLCPP_INFO(rclcpp::get_logger("SayTextClient"), "Saying: '%s'", text.c_str());
  
  hri_client_->start_speaking(text);
  start_time_ = std::chrono::steady_clock::now();
  
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus SayTextClientAction::onRunning() {
  auto elapsed = std::chrono::steady_clock::now() - start_time_;
  if (elapsed > std::chrono::seconds(30)) {
    RCLCPP_ERROR(rclcpp::get_logger("SayTextClient"), "TTS timeout");
    return BT::NodeStatus::FAILURE;
  }
  
  if (hri_client_->is_speaking_done()) {
    bool success = hri_client_->get_speaking_result();
    if (success) {
      RCLCPP_INFO(rclcpp::get_logger("SayTextClient"), "TTS completed");
    } else {
      RCLCPP_ERROR(rclcpp::get_logger("SayTextClient"), "TTS failed");
    }
    return success ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }
  
  return BT::NodeStatus::RUNNING;
}

void SayTextClientAction::onHalted() {
  RCLCPP_INFO(rclcpp::get_logger("SayTextClient"), "TTS action halted");
}