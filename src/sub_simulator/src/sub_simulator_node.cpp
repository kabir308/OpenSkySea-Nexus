#include "rclcpp/rclcpp.hpp"
#include "hybrid_msgs/msg/vehicle_state.hpp"

class SubSimulator : public rclcpp::Node
{
public:
  SubSimulator()
  : Node("sub_simulator")
  {
    publisher_ = this->create_publisher<hybrid_msgs::msg::VehicleState>("sub/vehicle_state_out", 10);
    subscription_ = this->create_subscription<hybrid_msgs::msg::VehicleState>(
      "sub/vehicle_state_in", 10, std::bind(&SubSimulator::topic_callback, this, std::placeholders::_1));

    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100), std::bind(&SubSimulator::publish_state, this));
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

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<hybrid_msgs::msg::VehicleState>::SharedPtr publisher_;
  rclcpp::Subscription<hybrid_msgs::msg::VehicleState>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SubSimulator>());
  rclcpp::shutdown();
  return 0;
}
