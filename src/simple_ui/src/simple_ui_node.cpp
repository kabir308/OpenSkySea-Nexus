#include "rclcpp/rclcpp.hpp"
#include "nexus_communication_protocol/msg/go_to_waypoint.hpp"
#include <iostream>

class SimpleUI : public rclcpp::Node
{
public:
  SimpleUI()
  : Node("simple_ui")
  {
    publisher_ = this->create_publisher<nexus_communication_protocol::msg::GoToWaypoint>("ncp/go_to_waypoint", 10);
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(500), std::bind(&SimpleUI::prompt_user, this));
  }

private:
  void prompt_user()
  {
    std::cout << "Enter waypoint (x y): ";
    double x, y;
    std::cin >> x >> y;

    auto message = nexus_communication_protocol::msg::GoToWaypoint();
    message.header.stamp = this->now();
    message.position.x = x;
    message.position.y = y;
    publisher_->publish(message);
  }

  rclcpp::Publisher<nexus_communication_protocol::msg::GoToWaypoint>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SimpleUI>());
  rclcpp::shutdown();
  return 0;
}
