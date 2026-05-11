#include "waiter_robot/listen_text_client_action.hpp"
#include "hri_example/hri_client.hpp"

ListenTextClientAction::ListenTextClientAction(
    const std::string& name,
    const BT::NodeConfiguration& config,
    std::shared_ptr<HRIClient> hri_client)
  : BT::StatefulActionNode(name, config), hri_client_(hri_client)
{
}

BT::PortsList ListenTextClientAction::providedPorts() {
  return { BT::OutputPort<std::string>("recognized_text", "Recognized text") };
}

BT::NodeStatus ListenTextClientAction::onStart() {
  RCLCPP_INFO(rclcpp::get_logger("ListenTextClient"), "Listening for speech...");
  
  hri_client_->start_listen();
  start_time_ = std::chrono::steady_clock::now();
  
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus ListenTextClientAction::onRunning() {
  auto elapsed = std::chrono::steady_clock::now() - start_time_;
  if (elapsed > std::chrono::seconds(60)) {
    RCLCPP_ERROR(rclcpp::get_logger("ListenTextClient"), "STT timeout");
    return BT::NodeStatus::FAILURE;
  }
  
  if (hri_client_->is_listen_done()) {
    std::string recognized_text = hri_client_->get_listened_text();
    
    if (recognized_text.empty()) {
      RCLCPP_ERROR(rclcpp::get_logger("ListenTextClient"), "No speech recognized");
      return BT::NodeStatus::FAILURE;
    }
    
    RCLCPP_INFO(rclcpp::get_logger("ListenTextClient"), 
                "Recognized: '%s'", recognized_text.c_str());
    setOutput("recognized_text", recognized_text);
    
    return BT::NodeStatus::SUCCESS;
  }
  
  return BT::NodeStatus::RUNNING;
}

void ListenTextClientAction::onHalted() {
  RCLCPP_INFO(rclcpp::get_logger("ListenTextClient"), "STT action halted");
}