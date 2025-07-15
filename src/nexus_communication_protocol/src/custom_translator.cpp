#include "nexus_communication_protocol/translator_plugin.hpp"
#include "std_msgs/msg/string.hpp"
#include <pluginlib/class_list_macros.hpp>

namespace nexus_communication_protocol
{

class CustomTranslator : public TranslatorPlugin
{
public:
  void initialize(rclcpp::Node::SharedPtr node) override
  {
    node_ = node;
    publisher_ = node_->create_publisher<std_msgs::msg::String>("custom_autopilot/command", 10);
  }

  void translate(const nexus_communication_protocol::msg::GoToWaypoint::SharedPtr msg) override
  {
    RCLCPP_INFO(node_->get_logger(), "Translating GoToWaypoint message for custom autopilot");
    auto command = std_msgs::msg::String();
    command.data = "GOTO " + std::to_string(msg->position.x) + "," + std::to_string(msg->position.y);
    publisher_->publish(command);
  }

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
};

}  // namespace nexus_communication_protocol

PLUGINLIB_EXPORT_CLASS(nexus_communication_protocol::CustomTranslator, nexus_communication_protocol::TranslatorPlugin)
