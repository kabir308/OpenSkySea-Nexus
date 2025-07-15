#include "nexus_communication_protocol/translator_plugin.hpp"
#include "std_msgs/msg/string.hpp"
#include "nexus_communication_protocol/msg/follow_trajectory.hpp"
#include <pluginlib/class_list_macros.hpp>

namespace nexus_communication_protocol
{

class PyPilotTranslator : public TranslatorPlugin
{
public:
  void initialize(rclcpp::Node::SharedPtr node) override
  {
    node_ = node;
    command_publisher_ = node_->create_publisher<std_msgs::msg::String>("pypilot/command", 10);
    follow_trajectory_subscription_ = node_->create_subscription<nexus_communication_protocol::msg::FollowTrajectory>(
      "ncp/follow_trajectory", 10, std::bind(&PyPilotTranslator::follow_trajectory_callback, this, std::placeholders::_1));
  }

  void translate(const nexus_communication_protocol::msg::GoToWaypoint::SharedPtr msg) override
  {
    RCLCPP_INFO(node_->get_logger(), "Translating GoToWaypoint message for PyPilot");
    auto pypilot_command = std_msgs::msg::String();
    pypilot_command.data = "waypoint " + std::to_string(msg->position.x) + " " + std::to_string(msg->position.y);
    command_publisher_->publish(pypilot_command);
  }

private:
  void follow_trajectory_callback(const nexus_communication_protocol::msg::FollowTrajectory::SharedPtr msg) const
  {
    RCLCPP_INFO(node_->get_logger(), "Translating FollowTrajectory message for PyPilot");
    // TODO: Implement trajectory following logic for PyPilot
  }

  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr command_publisher_;
  rclcpp::Subscription<nexus_communication_protocol::msg::FollowTrajectory>::SharedPtr follow_trajectory_subscription_;
};

}  // namespace nexus_communication_protocol

PLUGINLIB_EXPORT_CLASS(nexus_communication_protocol::PyPilotTranslator, nexus_communication_protocol::TranslatorPlugin)
