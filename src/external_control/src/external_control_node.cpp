#include "rclcpp/rclcpp.hpp"
#include "httplib.h"
#include "nexus_communication_protocol/msg/go_to_waypoint.hpp"

class ExternalControl : public rclcpp::Node
{
public:
  ExternalControl()
  : Node("external_control")
  {
    publisher_ = this->create_publisher<nexus_communication_protocol::msg::GoToWaypoint>("ncp/go_to_waypoint", 10);

    server_thread_ = std::thread([this]() {
      httplib::Server svr;

      svr.Post("/waypoint", [this](const httplib::Request &req, httplib::Response &res) {
        // TODO: Add error handling
        double x = std::stod(req.get_param_value("x"));
        double y = std::stod(req.get_param_value("y"));

        auto message = nexus_communication_protocol::msg::GoToWaypoint();
        message.header.stamp = this->now();
        message.position.x = x;
        message.position.y = y;
        publisher_->publish(message);

        res.set_content("Waypoint set", "text/plain");
      });

      svr.listen("0.0.0.0", 8080);
    });
  }

  ~ExternalControl()
  {
    if (server_thread_.joinable()) {
      server_thread_.join();
    }
  }

private:
  rclcpp::Publisher<nexus_communication_protocol::msg::GoToWaypoint>::SharedPtr publisher_;
  std::thread server_thread_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ExternalControl>());
  rclcpp::shutdown();
  return 0;
}
