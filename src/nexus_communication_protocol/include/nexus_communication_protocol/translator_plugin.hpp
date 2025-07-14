#ifndef NEXUS_COMMUNICATION_PROTOCOL__TRANSLATOR_PLUGIN_HPP_
#define NEXUS_COMMUNICATION_PROTOCOL__TRANSLATOR_PLUGIN_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nexus_communication_protocol/msg/go_to_waypoint.hpp"

namespace nexus_communication_protocol
{

class TranslatorPlugin
{
public:
  virtual ~TranslatorPlugin() {}
  virtual void initialize(rclcpp::Node::SharedPtr node) = 0;
  virtual void translate(const nexus_communication_protocol::msg::GoToWaypoint::SharedPtr msg) = 0;
};

}  // namespace nexus_communication_protocol

#endif  // NEXUS_COMMUNICATION_PROTOCOL__TRANSLATOR_PLUGIN_HPP_
