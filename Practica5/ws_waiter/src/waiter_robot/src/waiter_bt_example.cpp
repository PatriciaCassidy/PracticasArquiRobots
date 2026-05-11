#include <rclcpp/rclcpp.hpp>
#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/loggers/bt_cout_logger.h>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include "waiter_robot/bt_nodes/bt_node_registration.hpp"
#include "hri_example/hri_client.hpp"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("waiter_bt");
  
  // Create HRIClient for TTS/STT/LLM services
  auto hri_client = std::make_shared<HRIClient>(node);
  
  // Factory for registering custom nodes
  BT::BehaviorTreeFactory factory;
  register_waiter_nodes(factory, node, hri_client);
  
  // Get path to XML file
  std::string package_share_dir = 
      ament_index_cpp::get_package_share_directory("waiter_robot");
  std::string tree_path = package_share_dir + "/config/waiter_tree.xml";
  
  RCLCPP_INFO(node->get_logger(), "Loading behavior tree from: %s", tree_path.c_str());
  
  // Create blackboard and set resources BEFORE creating the tree
  auto blackboard = BT::Blackboard::create();
  blackboard->set("node", node);
  blackboard->set("hri_client", hri_client);
  
  // Set default locations
  node->declare_parameter("kitchen_x", 2.0);
  node->declare_parameter("kitchen_y", 5.0);
  node->declare_parameter("kitchen_yaw", 0.0);
  node->declare_parameter("client_x", 2.0);
  node->declare_parameter("client_y", -2.0);
  node->declare_parameter("client_yaw", 0.0);
  node->declare_parameter("home_x", -2.0);
  node->declare_parameter("home_y", 0.0);
  node->declare_parameter("home_yaw", 0.0);
  
  double kitchen_x = node->get_parameter("kitchen_x").as_double();
  double kitchen_y = node->get_parameter("kitchen_y").as_double();
  double kitchen_yaw = node->get_parameter("kitchen_yaw").as_double();
  double client_x = node->get_parameter("client_x").as_double();
  double client_y = node->get_parameter("client_y").as_double();
  double client_yaw = node->get_parameter("client_yaw").as_double();
  double home_x = node->get_parameter("home_x").as_double();
  double home_y = node->get_parameter("home_y").as_double();
  double home_yaw = node->get_parameter("home_yaw").as_double();
  
  blackboard->set("kitchen_x", kitchen_x);
  blackboard->set("kitchen_y", kitchen_y);
  blackboard->set("kitchen_yaw", kitchen_yaw);
  blackboard->set("client_x", client_x);
  blackboard->set("client_y", client_y);
  blackboard->set("client_yaw", client_yaw);
  blackboard->set("home_x", home_x);
  blackboard->set("home_y", home_y);
  blackboard->set("home_yaw", home_yaw);
  
  // Load tree from XML with the blackboard containing resources
  auto tree = factory.createTreeFromFile(tree_path, blackboard);
  
  // Logger for debugging
  BT::StdCoutLogger logger(tree);
  
  RCLCPP_INFO(node->get_logger(), "Waiter robot behavior tree started");
  RCLCPP_INFO(node->get_logger(), "Kitchen: (%.2f, %.2f)", kitchen_x, kitchen_y);
  RCLCPP_INFO(node->get_logger(), "Client: (%.2f, %.2f)", client_x, client_y);
  RCLCPP_INFO(node->get_logger(), "Home: (%.2f, %.2f)", home_x, home_y);
  
  // Main loop: tick the tree at 10 Hz
  rclcpp::Rate rate(10);
  BT::NodeStatus status = BT::NodeStatus::IDLE;
  
  while (rclcpp::ok() && status != BT::NodeStatus::FAILURE) {
    // Process ROS callbacks
    rclcpp::spin_some(node);
    
    // Evaluate tree
    status = tree.tickOnce();
    
    // Reset tree if it completes (for continuous execution)
    if (status == BT::NodeStatus::SUCCESS) {
      RCLCPP_INFO(node->get_logger(), "Mission completed, restarting...");
      tree.rootBlackboard()->set("recognized_text", "");
      tree.rootBlackboard()->set("bebida", "");
      tree.rootBlackboard()->set("comida", "");
      status = BT::NodeStatus::IDLE;
    }
    
    rate.sleep();
  }
  
  if (status == BT::NodeStatus::FAILURE) {
    RCLCPP_ERROR(node->get_logger(), "Mission failed");
  }
  
  rclcpp::shutdown();
  return 0;
}