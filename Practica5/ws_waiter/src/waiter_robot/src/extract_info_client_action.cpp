#include "waiter_robot/extract_info_client_action.hpp"
#include "hri_example/hri_client.hpp"

ExtractInfoClientAction::ExtractInfoClientAction(
    const std::string& name,
    const BT::NodeConfiguration& config,
    std::shared_ptr<HRIClient> hri_client)
  : BT::StatefulActionNode(name, config), hri_client_(hri_client)
{
}

BT::PortsList ExtractInfoClientAction::providedPorts() {
  return {
    BT::InputPort<std::string>("interest", "What to extract"),
    BT::InputPort<std::string>("full_text", "Full text to analyze"),
    BT::OutputPort<std::string>("extracted_info", "Extracted information")
  };
}

BT::NodeStatus ExtractInfoClientAction::onStart() {
  std::string interest, full_text;
  if (!getInput("interest", interest) || !getInput("full_text", full_text)) {
    RCLCPP_ERROR(rclcpp::get_logger("ExtractInfoClient"), 
                 "Missing required input ports");
    return BT::NodeStatus::FAILURE;
  }
  
  RCLCPP_INFO(rclcpp::get_logger("ExtractInfoClient"), 
              "Extracting '%s' from: '%s'", interest.c_str(), full_text.c_str());
  
  hri_client_->start_extract(interest, full_text);
  start_time_ = std::chrono::steady_clock::now();
  
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus ExtractInfoClientAction::onRunning() {
  auto elapsed = std::chrono::steady_clock::now() - start_time_;
  if (elapsed > std::chrono::seconds(10)) {
    RCLCPP_ERROR(rclcpp::get_logger("ExtractInfoClient"), "Extract timeout");
    return BT::NodeStatus::FAILURE;
  }
  
  if (hri_client_->is_extract_done()) {
    std::string extracted = hri_client_->get_extracted_info();
    if (extracted.empty()) {
      RCLCPP_WARN(rclcpp::get_logger("ExtractInfoClient"), 
                  "Extract returned empty result");
      return BT::NodeStatus::FAILURE;
    }
    
    RCLCPP_INFO(rclcpp::get_logger("ExtractInfoClient"), 
                "Extracted: '%s'", extracted.c_str());
    setOutput("extracted_info", extracted);
    
    return BT::NodeStatus::SUCCESS;
  }
  
  return BT::NodeStatus::RUNNING;
}

void ExtractInfoClientAction::onHalted() {
  RCLCPP_INFO(rclcpp::get_logger("ExtractInfoClient"), "Extract action halted");
}