#include "waiter_robot/bt_nodes/bt_node_registration.hpp"
#include "waiter_robot/navigate_to_pose_action.hpp"
#include "waiter_robot/say_text_client_action.hpp"
#include "waiter_robot/listen_text_client_action.hpp"
#include "waiter_robot/extract_info_client_action.hpp"

void register_waiter_nodes(
    BT::BehaviorTreeFactory& factory,
    rclcpp::Node::SharedPtr node,
    std::shared_ptr<HRIClient> hri_client)
{
  // Register NavigateToPose action node
  factory.registerBuilder<NavigateToPoseAction>(
    "NavigateToPose",
    [node](const std::string& name, const BT::NodeConfiguration& config) {
      return std::make_unique<NavigateToPoseAction>(name, config, node);
    });

  // Register HRI nodes (only if HRIClient is available)
  if (hri_client) {
    factory.registerBuilder<SayTextClientAction>(
      "SayTextClient",
      [hri_client](const std::string& name, const BT::NodeConfiguration& config) {
        return std::make_unique<SayTextClientAction>(name, config, hri_client);
      });

    factory.registerBuilder<ListenTextClientAction>(
      "ListenTextClient",
      [hri_client](const std::string& name, const BT::NodeConfiguration& config) {
        return std::make_unique<ListenTextClientAction>(name, config, hri_client);
      });

    factory.registerBuilder<ExtractInfoClientAction>(
      "ExtractInfoClient",
      [hri_client](const std::string& name, const BT::NodeConfiguration& config) {
        return std::make_unique<ExtractInfoClientAction>(name, config, hri_client);
      });
  }
}