#include "waiter_robot/text_utils.hpp"
#include <behaviortree_cpp/blackboard.h>
#include <regex>
#include <rclcpp/rclcpp.hpp>  // ← AÑADIR ESTA LÍNEA

namespace waiter_robot {

std::string formatText(const std::string& text, BT::Blackboard::Ptr blackboard) {
  std::string result = text;
  
  // Replace {variable_name} with values from blackboard
  std::regex pattern("\\{([^}]+)\\}");
  std::smatch match;
  
  while (std::regex_search(result, match, pattern)) {
    std::string variable_name = match[1].str();
    std::string full_match = match[0].str();
    
    try {
      // Try to get the value from blackboard
      std::string value = blackboard->get<std::string>(variable_name);
      result.replace(match.position(), full_match.length(), value);
    } catch (...) {
      RCLCPP_WARN(rclcpp::get_logger("TextUtils"), 
                  "Variable '%s' not found in blackboard, keeping original", 
                  variable_name.c_str());
      break; // Stop to avoid infinite loop if variable doesn't exist
    }
  }
  
  return result;
}

} // namespace waiter_robot