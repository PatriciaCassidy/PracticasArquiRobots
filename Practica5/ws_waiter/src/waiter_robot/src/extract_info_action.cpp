// src/extract_info_action.cpp

#include "waiter_robot/extract_info_action.hpp"
#include <algorithm>
#include <cctype>

namespace
{
// Convierte a minusculas
std::string toLower(std::string s)
{
  std::transform(s.begin(), s.end(), s.begin(),
    [](unsigned char c) {return std::tolower(c);});
  return s;
}

// Devuelve true si 'text' contiene 'keyword'
bool contains(const std::string & text, const std::string & keyword)
{
  return text.find(keyword) != std::string::npos;
}
}  // namespace

ExtractInfoAction::ExtractInfoAction(
  const std::string & name,
  const BT::NodeConfiguration & config,
  rclcpp::Node::SharedPtr node)
: BT::SyncActionNode(name, config), node_(node)
{
}

BT::PortsList ExtractInfoAction::providedPorts()
{
  return {
    BT::InputPort<std::string>("interest",       "Category to extract: 'bebida' or 'comida'"),
    BT::InputPort<std::string>("full_text",      "Full text to analyse"),
    BT::OutputPort<std::string>("extracted_info","Extracted item")
  };
}

BT::NodeStatus ExtractInfoAction::tick()
{
  std::string interest, full_text;
  if (!getInput("interest", interest) || !getInput("full_text", full_text)) {
    RCLCPP_ERROR(node_->get_logger(), "ExtractInfo: missing required input ports");
    return BT::NodeStatus::FAILURE;
  }

  const std::string text = toLower(full_text);
  std::string extracted = "desconocido";

  if (interest == "bebida") {
    if      (contains(text, "cafe") || contains(text, "café"))   extracted = "cafe";
    else if (contains(text, "te") || contains(text, "té"))        extracted = "te";
    else if (contains(text, "agua"))                              extracted = "agua";
    else if (contains(text, "zumo") || contains(text, "jugo"))    extracted = "zumo";
    else if (contains(text, "refresco") || contains(text, "cola"))extracted = "refresco";
    else if (contains(text, "cerveza"))                           extracted = "cerveza";
  } else if (interest == "comida") {
    if      (contains(text, "tostada"))                           extracted = "tostada";
    else if (contains(text, "bocadillo") || contains(text, "sandwich")) extracted = "bocadillo";
    else if (contains(text, "croissant"))                         extracted = "croissant";
    else if (contains(text, "pincho"))                            extracted = "pincho";
    else if (contains(text, "tortilla"))                          extracted = "tortilla";
    else if (contains(text, "napolitana"))                        extracted = "napolitana";
  }

  RCLCPP_INFO(node_->get_logger(),
    "[ExtractInfo] interest='%s'  text='%s'  -> '%s'",
    interest.c_str(), full_text.c_str(), extracted.c_str());

  setOutput("extracted_info", extracted);
  return BT::NodeStatus::SUCCESS;
}