// src/text_utils.cpp
#include "waiter_robot/text_utils.hpp"
#include <regex>
#include <rclcpp/rclcpp.hpp>

namespace waiter_robot
{

std::string formatText(const std::string & text, BT::Blackboard::Ptr blackboard)
{
  std::string result = text;
  std::regex pattern("\\{([^}]+)\\}");
  std::smatch match;

  // Iteramos sobre todas las ocurrencias {variable}
  while (std::regex_search(result, match, pattern)) {
    const std::string var_name  = match[1].str();
    const std::string full_match = match[0].str();

    try {
      std::string value = blackboard->get<std::string>(var_name);
      result.replace(match.position(), full_match.length(), value);
    } catch (...) {
      // La variable no existe en el blackboard: la dejamos como esta
      // y salimos para no entrar en bucle infinito
      RCLCPP_WARN(rclcpp::get_logger("TextUtils"),
        "Variable '%s' no encontrada en blackboard, se mantiene literal",
        var_name.c_str());
      break;
    }
  }

  return result;
}

}  // namespace waiter_robot