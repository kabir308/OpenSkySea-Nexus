#include "rclcpp/rclcpp.hpp"
#include "hybrid_msgs/msg/vehicle_state.hpp"
#include "energy_msgs/msg/power_state.hpp"

class SeaSimulator : public rclcpp::Node
{
public:
  SeaSimulator()
  : Node("sea_simulator")
  {
    state_publisher_ = this->create_publisher<hybrid_msgs::msg::VehicleState>("sea/vehicle_state_out", 10);
    power_publisher_ = this->create_publisher<energy_msgs::msg::PowerState>("sea/power_state_out", 10);

    subscription_ = this->create_subscription<hybrid_msgs::msg::VehicleState>(
      "sea/vehicle_state_in", 10, std::bind(&SeaSimulator::topic_callback, this, std::placeholders::_1));

    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100), std::bind(&SeaSimulator::publish_state, this));
  }

private:
  void topic_callback(const hybrid_msgs::msg::VehicleState::SharedPtr msg)
  {
    RCLCPP_INFO(this->get_logger(), "Received vehicle state");
    // TODO: Update internal state based on received message
  }

  void publish_state()
  {
    auto state_message = hybrid_msgs::msg::VehicleState();
    state_message.header.stamp = this->now();
    // TODO: Update with actual simulated state
    state_publisher_->publish(state_message);

    auto power_message = energy_msgs::msg::PowerState();
    power_message.header.stamp = this->now();
    // TODO: Update with actual simulated power consumption
    power_message.battery_current = -10.0; // 10A discharge
    power_publisher_->publish(power_message);
  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<hybrid_msgs::msg::VehicleState>::SharedPtr state_publisher_;
  rclcpp::Publisher<energy_msgs::msg::PowerState>::SharedPtr power_publisher_;
  rclcpp::Subscription<hybrid_msgs::msg::VehicleState>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SeaSimulator>());
  rclcpp::shutdown();
  return 0;
}
