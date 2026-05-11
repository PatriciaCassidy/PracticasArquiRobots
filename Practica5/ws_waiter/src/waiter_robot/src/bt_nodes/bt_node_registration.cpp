#include "waiter_robot/bt_nodes/bt_node_registration.hpp"
#include "waiter_robot/navigate_to_pose_action.hpp"
#include "waiter_robot/say_text_action.hpp"
#include "waiter_robot/listen_text_action.hpp"
#include "waiter_robot/extract_info_action.hpp"

void register_waiter_nodes(
    BT::BehaviorTreeFactory& factory,
    rclcpp::Node::SharedPtr node)
{
  factory.registerBuilder<NavigateToPoseAction>(
    "NavigateToPose",
    [node](const std::string& name, const BT::NodeConfiguration& config) {
      return std::make_unique<NavigateToPoseAction>(name, config, node);
    });

  factory.registerBuilder<SayTextAction>(
    "SayText",
    [node](const std::string& name, const BT::NodeConfiguration& config) {
      return std::make_unique<SayTextAction>(name, config, node);
    });

  factory.registerBuilder<ListenTextAction>(
    "ListenText",
    [node](const std::string& name, const BT::NodeConfiguration& config) {
      return std::make_unique<ListenTextAction>(name, config, node);
    });

  factory.registerBuilder<ExtractInfoAction>(
    "ExtractInfo",
    [node](const std::string& name, const BT::NodeConfiguration& config) {
      return std::make_unique<ExtractInfoAction>(name, config, node);
    });
}