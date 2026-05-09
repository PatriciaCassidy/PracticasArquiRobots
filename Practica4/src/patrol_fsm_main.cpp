#include "rclcpp/rclcpp.hpp"
#include "patrol_fsm/patrol_fsm_node.hpp"

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  
  auto node = std::make_shared<patrol_fsm::PatrolFSMNode>();
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  
  RCLCPP_INFO(node->get_logger(), "============================================");
  RCLCPP_INFO(node->get_logger(), "PATRULLAJE AUTONOMO INICIADO");
  RCLCPP_INFO(node->get_logger(), "============================================");
  
  executor.spin();
  rclcpp::shutdown();
  return 0;
}