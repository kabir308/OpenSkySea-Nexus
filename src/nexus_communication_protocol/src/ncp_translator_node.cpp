#include "rclcpp/rclcpp.hpp"
#include "nexus_communication_protocol/msg/go_to_waypoint.hpp"
#include "mavros_msgs/msg/position_target.hpp"

class NcpTranslator : public rclcpp::Node
{
public:
  NcpTranslator()
  : Node("ncp_translator")
  {
    publisher_ = this->create_publisher<mavros_msgs::msg::PositionTarget>("mavros/setpoint_raw/local", 10);
    subscription_ = this->create_subscription<nexus_communication_protocol::msg::GoToWaypoint>(
      "ncp/go_to_waypoint", 10, std::bind(&NcpTranslator::topic_callback, this, std::placeholders::_1));
  }

private:
  void topic_callback(const nexus_communication_protocol::msg::GoToWaypoint::SharedPtr msg) const
  {
    RCLCPP_INFO(this->get_logger(), "Received GoToWaypoint message");
    auto position_target = mavros_msgs::msg::PositionTarget();
    position_target.header.stamp = this->now();
    position_target.coordinate_frame = mavros_msgs::msg::PositionTarget::FRAME_LOCAL_NED;
    position_target.type_mask = mavros_msgs::msg::PositionTarget::IGNORE_VX |
                                mavros_msgs::msg::PositionTarget::IGNORE_VY |
                                mavros_msgs::msg::PositionTarget::IGNORE_VZ |
                                mavros_msgs::msg::PositionTarget::IGNORE_AFX |
                                mavros_msgs::msg::PositionTarget::IGNORE_AFY |
                                mavros_msgs::msg::PositionTarget::IGNORE_AFZ |
                                mavros_msgs::msg::PositionTarget::IGNORE_YAW_RATE;
    position_target.position = msg->position;
    publisher_->publish(position_target);
  }
  rclcpp::Subscription<nexus_communication_protocol::msg::GoToWaypoint>::SharedPtr subscription_;
  rclcpp::Publisher<mavros_msgs::msg::PositionTarget>::SharedPtr publisher_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<NcpTranslator>());
  rclcpp::shutdown();
  return 0;
}
