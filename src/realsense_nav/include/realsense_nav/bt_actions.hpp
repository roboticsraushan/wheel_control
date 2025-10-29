#ifndef REALSENSE_NAV__BT_ACTIONS_HPP_
#define REALSENSE_NAV__BT_ACTIONS_HPP_

#include <behaviortree_cpp_v3/action_node.h>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <chrono>
#include <cmath>

namespace realsense_nav
{

// Shared blackboard data
struct SharedData
{
  rclcpp::Node::SharedPtr node;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr goal_sub;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr goal_detected_sub;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr state_complete_sub;
  
  geometry_msgs::msg::Point goal_position;
  bool goal_detected = false;
  bool state_complete = false;
  std::chrono::steady_clock::time_point last_goal_time;
};

// Action: Go to Yellow Cone
class GoToYellowCone : public BT::StatefulActionNode
{
public:
  GoToYellowCone(const std::string& name, const BT::NodeConfiguration& config);

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<double>("stop_distance", 0.8, "Distance to stop from cone (meters)"),
      BT::InputPort<double>("timeout", 30.0, "Timeout in seconds")
    };
  }

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  std::chrono::steady_clock::time_point start_time_;
  SharedData* shared_data_;
};

// Action: Turn by specified degrees
class TurnDegrees : public BT::StatefulActionNode
{
public:
  TurnDegrees(const std::string& name, const BT::NodeConfiguration& config);

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<double>("degrees", "Degrees to turn (negative=right, positive=left)"),
      BT::InputPort<double>("angular_speed", 0.5, "Angular speed in rad/s")
    };
  }

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  std::chrono::steady_clock::time_point start_time_;
  double expected_duration_;
  double angular_speed_;
  SharedData* shared_data_;
};

// Action: Wait for duration
class Wait : public BT::StatefulActionNode
{
public:
  Wait(const std::string& name, const BT::NodeConfiguration& config);

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<double>("duration", "Wait duration in seconds")
    };
  }

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  std::chrono::steady_clock::time_point start_time_;
  SharedData* shared_data_;
};

} // namespace realsense_nav

#endif // REALSENSE_NAV__BT_ACTIONS_HPP_
