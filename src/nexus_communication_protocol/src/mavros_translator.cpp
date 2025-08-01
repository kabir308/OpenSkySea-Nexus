#include "nexus_communication_protocol/translator_plugin.hpp"
#include "mavros_msgs/msg/position_target.hpp"
#include "nexus_communication_protocol/msg/follow_trajectory.hpp"
#include <pluginlib/class_list_macros.hpp>

namespace nexus_communication_protocol
{

class MavrosTranslator : public TranslatorPlugin
{
public:
  void initialize(rclcpp::Node::SharedPtr node) override
  {
    node_ = node;
    position_target_publisher_ = node_->create_publisher<mavros_msgs::msg::PositionTarget>("mavros/setpoint_raw/local", 10);
    follow_trajectory_subscription_ = node_->create_subscription<nexus_communication_protocol::msg::FollowTrajectory>(
      "ncp/follow_trajectory", 10, std::bind(&MavrosTranslator::follow_trajectory_callback, this, std::placeholders::_1));
  }

  void translate(const nexus_communication_protocol::msg::GoToWaypoint::SharedPtr msg) override
  {
    RCLCPP_INFO(node_->get_logger(), "Translating GoToWaypoint message for MAVROS");
    auto position_target = mavros_msgs::msg::PositionTarget();
    position_target.header.stamp = node_->now();
    position_target.coordinate_frame = mavros_msgs::msg::PositionTarget::FRAME_LOCAL_NED;
    position_target.type_mask = mavros_msgs::msg::PositionTarget::IGNORE_VX |
                                mavros_msgs::msg::PositionTarget::IGNORE_VY |
                                mavros_msgs::msg::PositionTarget::IGNORE_VZ |
                                mavros_msgs::msg::PositionTarget::IGNORE_AFX |
                                mavros_msgs::msg::PositionTarget::IGNORE_AFY |
                                mavros_msgs::msg::PositionTarget::IGNORE_AFZ |
                                mavros_msgs::msg::PositionTarget::IGNORE_YAW_RATE;
    position_target.position = msg->position;
    position_target_publisher_->publish(position_target);
  }

private:
  void follow_trajectory_callback(const nexus_communication_protocol::msg::FollowTrajectory::SharedPtr msg) const
  {
    RCLCPP_INFO(node_->get_logger(), "Translating FollowTrajectory message for MAVROS");
    // TODO: Implement trajectory following logic for MAVROS
  }

  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<mavros_msgs::msg::PositionTarget>::SharedPtr position_target_publisher_;
  rclcpp::Subscription<nexus_communication_protocol::msg::FollowTrajectory>::SharedPtr follow_trajectory_subscription_;
};

}  // namespace nexus_communication_protocol

PLUGINLIB_EXPORT_CLASS(nexus_communication_protocol::MavrosTranslator, nexus_communication_protocol::TranslatorPlugin)
