// src/waiter_bt_example.cpp

#include <rclcpp/rclcpp.hpp>
#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/loggers/bt_cout_logger.h>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include "waiter_robot/bt_nodes/bt_node_registration.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("waiter_bt");

  // --- Fabrica y registro de nodos ----------------------------------------
  BT::BehaviorTreeFactory factory;
  register_waiter_nodes(factory, node);

  // --- Localizar el XML del arbol ------------------------------------------
  const std::string pkg_share =
    ament_index_cpp::get_package_share_directory("waiter_robot");
  const std::string tree_path = pkg_share + "/config/waiter_tree.xml";

  RCLCPP_INFO(node->get_logger(), "Cargando BT desde: %s", tree_path.c_str());

  // --- Blackboard y parametros ---------------------------------------------
  auto blackboard = BT::Blackboard::create();

  node->declare_parameter("kitchen_x",   2.0);
  node->declare_parameter("kitchen_y",   5.0);
  node->declare_parameter("kitchen_yaw", 0.0);
  node->declare_parameter("client_x",    2.0);
  node->declare_parameter("client_y",   -2.0);
  node->declare_parameter("client_yaw",  0.0);
  node->declare_parameter("home_x",     -2.0);
  node->declare_parameter("home_y",      0.0);
  node->declare_parameter("home_yaw",    0.0);

  const double kitchen_x   = node->get_parameter("kitchen_x").as_double();
  const double kitchen_y   = node->get_parameter("kitchen_y").as_double();
  const double kitchen_yaw = node->get_parameter("kitchen_yaw").as_double();
  const double client_x    = node->get_parameter("client_x").as_double();
  const double client_y    = node->get_parameter("client_y").as_double();
  const double client_yaw  = node->get_parameter("client_yaw").as_double();
  const double home_x      = node->get_parameter("home_x").as_double();
  const double home_y      = node->get_parameter("home_y").as_double();
  const double home_yaw    = node->get_parameter("home_yaw").as_double();

  blackboard->set("kitchen_x",   kitchen_x);
  blackboard->set("kitchen_y",   kitchen_y);
  blackboard->set("kitchen_yaw", kitchen_yaw);
  blackboard->set("client_x",    client_x);
  blackboard->set("client_y",    client_y);
  blackboard->set("client_yaw",  client_yaw);
  blackboard->set("home_x",      home_x);
  blackboard->set("home_y",      home_y);
  blackboard->set("home_yaw",    home_yaw);

  // Inicializar variables de salida de los nodos BT
  blackboard->set<std::string>("recognized_text", "");
  blackboard->set<std::string>("bebida", "");
  blackboard->set<std::string>("comida", "");

  // --- Crear arbol ---------------------------------------------------------
  auto tree = factory.createTreeFromFile(tree_path, blackboard);
  BT::StdCoutLogger logger(tree);

  RCLCPP_INFO(node->get_logger(), "Robot camarero iniciado");
  RCLCPP_INFO(node->get_logger(), "  Home:    (%.1f, %.1f)", home_x,    home_y);
  RCLCPP_INFO(node->get_logger(), "  Cliente: (%.1f, %.1f)", client_x,  client_y);
  RCLCPP_INFO(node->get_logger(), "  Cocina:  (%.1f, %.1f)", kitchen_x, kitchen_y);

  // --- Bucle principal -----------------------------------------------------
  rclcpp::Rate rate(10);   // 10 Hz

  while (rclcpp::ok()) {
    rclcpp::spin_some(node);

    const BT::NodeStatus status = tree.tickOnce();

    if (status == BT::NodeStatus::SUCCESS) {
      RCLCPP_INFO(node->get_logger(), "Mision completada. Reiniciando ciclo...");
      // FIX: es obligatorio llamar a haltTree() antes de volver a tickear
      tree.haltTree();
      // Limpiar outputs del blackboard para el siguiente ciclo
      blackboard->set<std::string>("recognized_text", "");
      blackboard->set<std::string>("bebida", "");
      blackboard->set<std::string>("comida", "");

    } else if (status == BT::NodeStatus::FAILURE) {
      RCLCPP_ERROR(node->get_logger(), "Mision fallida. Reiniciando ciclo...");
      tree.haltTree();
      blackboard->set<std::string>("recognized_text", "");
      blackboard->set<std::string>("bebida", "");
      blackboard->set<std::string>("comida", "");
    }

    rate.sleep();
  }

  rclcpp::shutdown();
  return 0;
}