#include "rclcpp/rclcpp.hpp"
#include "hybrid_msgs/msg/vehicle_state.hpp"
#include "micro_drone_simulator/srv/recharge.hpp"

class MicroDroneSimulator : public rclcpp::Node
{
public:
  MicroDroneSimulator()
  : Node("micro_drone_simulator")
  {
    publisher_ = this->create_publisher<hybrid_msgs::msg::VehicleState>("hackflight/vehicle_state_out", 10);
    subscription_ = this->create_subscription<hybrid_msgs::msg::VehicleState>(
      "hackflight/vehicle_state_in", 10, std::bind(&MicroDroneSimulator::topic_callback, this, std::placeholders::_1));

    recharge_service_ = this->create_service<micro_drone_simulator::srv::Recharge>(
      "recharge", std::bind(&MicroDroneSimulator::recharge, this, std::placeholders::_1, std::placeholders::_2));

    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100), std::bind(&MicroDroneSimulator::publish_state, this));
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

  void recharge(
    const std::shared_ptr<micro_drone_simulator::srv::Recharge::Request> request,
    std::shared_ptr<micro_drone_simulator::srv::Recharge::Response>      response)
  {
    RCLCPP_INFO(this->get_logger(), "Recharging");
    response->success = true;
  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<hybrid_msgs::msg::VehicleState>::SharedPtr publisher_;
  rclcpp::Subscription<hybrid_msgs::msg::VehicleState>::SharedPtr subscription_;
  rclcpp::Service<micro_drone_simulator::srv::Recharge>::SharedPtr recharge_service_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MicroDroneSimulator>());
  rclcpp::shutdown();
  return 0;
}
