#include <rclcpp/rclcpp.hpp>
#include <behaviortree_cpp_v3/bt_factory.h>
#include <behaviortree_cpp_v3/loggers/bt_cout_logger.h>
#include <behaviortree_cpp_v3/loggers/bt_file_logger.h>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include "realsense_nav/bt_actions.hpp"

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  
  auto node = std::make_shared<rclcpp::Node>("behavior_tree_cpp_node");
  
  RCLCPP_INFO(node->get_logger(), "========================================");
  RCLCPP_INFO(node->get_logger(), "Behavior Tree Manager Node Starting...");
  RCLCPP_INFO(node->get_logger(), "Pure Pursuit handles navigation");
  RCLCPP_INFO(node->get_logger(), "Behavior Tree manages states");
  RCLCPP_INFO(node->get_logger(), "========================================");
  
  // Create shared data
  realsense_nav::SharedData shared_data;
  shared_data.node = node;
  shared_data.goal_detected = false;
  shared_data.state_complete = false;
  shared_data.last_goal_time = std::chrono::steady_clock::now();
  
  // Create publisher for behavior state (Pure Pursuit subscribes to this)
  shared_data.state_pub = node->create_publisher<std_msgs::msg::String>("/behavior_state", 10);
  
  // Create subscribers
  shared_data.goal_sub = node->create_subscription<geometry_msgs::msg::PointStamped>(
    "/goal/position", 10,
    [&shared_data](const geometry_msgs::msg::PointStamped::SharedPtr msg) {
      shared_data.goal_position = msg->point;
      shared_data.last_goal_time = std::chrono::steady_clock::now();
    });
  
  shared_data.goal_detected_sub = node->create_subscription<std_msgs::msg::Bool>(
    "/goal/detected", 10,
    [&shared_data](const std_msgs::msg::Bool::SharedPtr msg) {
      shared_data.goal_detected = msg->data;
      if (msg->data) {
        shared_data.last_goal_time = std::chrono::steady_clock::now();
      }
    });
  
  // Subscribe to state completion (Pure Pursuit publishes when state is done)
  shared_data.state_complete_sub = node->create_subscription<std_msgs::msg::Bool>(
    "/behavior_state_complete", 10,
    [&shared_data](const std_msgs::msg::Bool::SharedPtr msg) {
      shared_data.state_complete = msg->data;
      if (msg->data) {
        RCLCPP_INFO(shared_data.node->get_logger(), "State completed by Pure Pursuit");
      }
    });
  
  // Create BehaviorTree factory
  BT::BehaviorTreeFactory factory;
  
  // Register custom nodes
  factory.registerNodeType<realsense_nav::GoToYellowCone>("GoToYellowCone");
  factory.registerNodeType<realsense_nav::TurnDegrees>("TurnDegrees");
  factory.registerNodeType<realsense_nav::Wait>("Wait");
  
  // Get behavior tree XML file path
  std::string package_share_dir;
  try {
    package_share_dir = ament_index_cpp::get_package_share_directory("realsense_nav");
  } catch (const std::exception& e) {
    RCLCPP_ERROR(node->get_logger(), "Failed to get package share directory: %s", e.what());
    return 1;
  }
  
  std::string bt_xml_file = package_share_dir + "/behavior_trees/cone_navigation.xml";
  RCLCPP_INFO(node->get_logger(), "Loading behavior tree from: %s", bt_xml_file.c_str());
  
  // Load and create behavior tree
  BT::Tree tree;
  try {
    tree = factory.createTreeFromFile(bt_xml_file);
  } catch (const std::exception& e) {
    RCLCPP_ERROR(node->get_logger(), "Failed to load behavior tree: %s", e.what());
    return 1;
  }
  
  // Set shared data in blackboard
  tree.rootBlackboard()->set("shared_data", &shared_data);
  
  // Create logger for visualization
  BT::StdCoutLogger logger_cout(tree);
  
  RCLCPP_INFO(node->get_logger(), "========================================");
  RCLCPP_INFO(node->get_logger(), "Behavior Tree loaded successfully!");
  RCLCPP_INFO(node->get_logger(), "Publishing states to /behavior_state");
  RCLCPP_INFO(node->get_logger(), "========================================");
  
  // Tick the tree at 20Hz
  rclcpp::Rate rate(20);
  
  while (rclcpp::ok()) {
    rclcpp::spin_some(node);
    
    BT::NodeStatus status = tree.tickRoot();
    
    if (status == BT::NodeStatus::SUCCESS) {
      RCLCPP_INFO(node->get_logger(), "Behavior tree completed successfully!");
      
      // Publish STOP state
      std_msgs::msg::String state_msg;
      state_msg.data = "STOP";
      shared_data.state_pub->publish(state_msg);
      
      break;
    } else if (status == BT::NodeStatus::FAILURE) {
      RCLCPP_ERROR(node->get_logger(), "Behavior tree failed!");
      
      // Publish STOP state
      std_msgs::msg::String state_msg;
      state_msg.data = "STOP";
      shared_data.state_pub->publish(state_msg);
      
      break;
    }
    
    rate.sleep();
  }
  
  rclcpp::shutdown();
  return 0;
}
