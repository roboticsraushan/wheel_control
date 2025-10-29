#include "realsense_nav/bt_actions.hpp"

namespace realsense_nav
{

// ============================================================================
// GoToYellowCone Implementation
// ============================================================================

GoToYellowCone::GoToYellowCone(const std::string& name, const BT::NodeConfiguration& config)
  : BT::StatefulActionNode(name, config)
{
  shared_data_ = config.blackboard->get<SharedData*>("shared_data");
}

BT::NodeStatus GoToYellowCone::onStart()
{
  start_time_ = std::chrono::steady_clock::now();
  shared_data_->state_complete = false;
  
  double stop_distance;
  if (!getInput("stop_distance", stop_distance)) {
    stop_distance = 0.8;
  }
  
  RCLCPP_INFO(shared_data_->node->get_logger(), 
              "BT: GoToYellowCone (stop at %.2fm)", stop_distance);
  
  // Publish state command for pure pursuit controller
  std_msgs::msg::String state_msg;
  state_msg.data = "GO_TO_CONE:" + std::to_string(stop_distance);
  shared_data_->state_pub->publish(state_msg);
  
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus GoToYellowCone::onRunning()
{
  auto now = std::chrono::steady_clock::now();
  auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(now - start_time_).count();
  
  double timeout;
  if (!getInput("timeout", timeout)) {
    timeout = 30.0;
  }
  
  // Check timeout
  if (elapsed > timeout) {
    RCLCPP_WARN(shared_data_->node->get_logger(), "BT: GoToYellowCone timed out");
    return BT::NodeStatus::FAILURE;
  }
  
  // Check if goal reached (pure pursuit controller will set this)
  if (shared_data_->state_complete) {
    RCLCPP_INFO(shared_data_->node->get_logger(), "BT: GoToYellowCone completed");
    return BT::NodeStatus::SUCCESS;
  }
  
  return BT::NodeStatus::RUNNING;
}

void GoToYellowCone::onHalted()
{
  std_msgs::msg::String state_msg;
  state_msg.data = "STOP";
  shared_data_->state_pub->publish(state_msg);
  
  RCLCPP_INFO(shared_data_->node->get_logger(), "BT: GoToYellowCone halted");
}

// ============================================================================
// TurnDegrees Implementation
// ============================================================================

TurnDegrees::TurnDegrees(const std::string& name, const BT::NodeConfiguration& config)
  : BT::StatefulActionNode(name, config)
{
  shared_data_ = config.blackboard->get<SharedData*>("shared_data");
}

BT::NodeStatus TurnDegrees::onStart()
{
  start_time_ = std::chrono::steady_clock::now();
  shared_data_->state_complete = false;
  
  double degrees;
  if (!getInput("degrees", degrees)) {
    RCLCPP_ERROR(shared_data_->node->get_logger(), "BT: Missing 'degrees' input");
    return BT::NodeStatus::FAILURE;
  }
  
  if (!getInput("angular_speed", angular_speed_)) {
    angular_speed_ = 0.5;
  }
  
  double radians = degrees * M_PI / 180.0;
  expected_duration_ = std::abs(radians) / angular_speed_;
  
  RCLCPP_INFO(shared_data_->node->get_logger(), 
              "BT: TurnDegrees(%.1f°) - Expected: %.2fs", 
              degrees, expected_duration_);
  
  // Publish state command for pure pursuit controller
  std_msgs::msg::String state_msg;
  if (degrees > 0) {
    state_msg.data = "TURN_LEFT:" + std::to_string(degrees);
  } else {
    state_msg.data = "TURN_RIGHT:" + std::to_string(std::abs(degrees));
  }
  shared_data_->state_pub->publish(state_msg);
  
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus TurnDegrees::onRunning()
{
  auto now = std::chrono::steady_clock::now();
  auto elapsed = std::chrono::duration<double>(now - start_time_).count();
  
  if (elapsed >= expected_duration_ || shared_data_->state_complete) {
    RCLCPP_INFO(shared_data_->node->get_logger(), 
                "BT: TurnDegrees completed in %.2fs", elapsed);
    return BT::NodeStatus::SUCCESS;
  }
  
  return BT::NodeStatus::RUNNING;
}

void TurnDegrees::onHalted()
{
  std_msgs::msg::String state_msg;
  state_msg.data = "STOP";
  shared_data_->state_pub->publish(state_msg);
  
  RCLCPP_INFO(shared_data_->node->get_logger(), "BT: TurnDegrees halted");
}

// ============================================================================
// Wait Implementation
// ============================================================================

Wait::Wait(const std::string& name, const BT::NodeConfiguration& config)
  : BT::StatefulActionNode(name, config)
{
  shared_data_ = config.blackboard->get<SharedData*>("shared_data");
}

BT::NodeStatus Wait::onStart()
{
  start_time_ = std::chrono::steady_clock::now();
  
  double duration;
  if (!getInput("duration", duration)) {
    RCLCPP_ERROR(shared_data_->node->get_logger(), "BT: Missing 'duration' input");
    return BT::NodeStatus::FAILURE;
  }
  
  // Publish WAIT state
  std_msgs::msg::String state_msg;
  state_msg.data = "WAIT:" + std::to_string(duration);
  shared_data_->state_pub->publish(state_msg);
  
  RCLCPP_INFO(shared_data_->node->get_logger(), "BT: Wait(%.1fs)", duration);
  
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus Wait::onRunning()
{
  auto now = std::chrono::steady_clock::now();
  auto elapsed = std::chrono::duration<double>(now - start_time_).count();
  
  double duration;
  getInput("duration", duration);
  
  if (elapsed >= duration) {
    RCLCPP_INFO(shared_data_->node->get_logger(), "BT: Wait completed");
    return BT::NodeStatus::SUCCESS;
  }
  
  return BT::NodeStatus::RUNNING;
}

void Wait::onHalted()
{
  RCLCPP_INFO(shared_data_->node->get_logger(), "BT: Wait halted");
}

} // namespace realsense_nav
