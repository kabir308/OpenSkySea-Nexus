#include "rclcpp/rclcpp.hpp"
#include "hybrid_msgs/msg/vehicle_state.hpp"
#include "vtol_simulator/srv/drop_buoy.hpp"

class VtolSimulator : public rclcpp::Node
{
public:
  VtolSimulator()
  : Node("vtol_simulator")
  {
    publisher_ = this->create_publisher<hybrid_msgs::msg::VehicleState>("betaflight/vehicle_state_out", 10);
    subscription_ = this->create_subscription<hybrid_msgs::msg::VehicleState>(
      "betaflight/vehicle_state_in", 10, std::bind(&VtolSimulator::topic_callback, this, std::placeholders::_1));

    drop_buoy_service_ = this->create_service<vtol_simulator::srv::DropBuoy>(
      "drop_buoy", std::bind(&VtolSimulator::drop_buoy, this, std::placeholders::_1, std::placeholders::_2));

    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100), std::bind(&VtolSimulator::publish_state, this));
  }

private:
  void topic_callback(const hybrid_msgs::msg::VehicleState::SharedPtr msg)
  {
    RCLCPP_INFO(this->get_logger(), "Received vehicle state");
    // TODO: Update internal state based on received message
  }

  void publish_state()
  {
    auto message = hybrid_msgs::msg::VehicleState();
    message.header.stamp = this->now();
    // TODO: Update with actual simulated state
    publisher_->publish(message);
  }

  void drop_buoy(
    const std::shared_ptr<vtol_simulator::srv::DropBuoy::Request> request,
    std::shared_ptr<vtol_simulator::srv::DropBuoy::Response>      response)
  {
    RCLCPP_INFO(this->get_logger(), "Dropping buoy");
    response->success = true;
  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<hybrid_msgs::msg::VehicleState>::SharedPtr publisher_;
  rclcpp::Subscription<hybrid_msgs::msg::VehicleState>::SharedPtr subscription_;
  rclcpp::Service<vtol_simulator::srv::DropBuoy>::SharedPtr drop_buoy_service_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<VtolSimulator>());
  rclcpp::shutdown();
  return 0;
}
