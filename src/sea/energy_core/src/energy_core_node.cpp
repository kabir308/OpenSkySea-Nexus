#include "rclcpp/rclcpp.hpp"
#include "energy_msgs/msg/power_state.hpp"

class EnergyCore : public rclcpp::Node
{
public:
  EnergyCore()
  : Node("energy_core")
  {
    publisher_ = this->create_publisher<energy_msgs::msg::PowerState>("power_state", 10);
    timer_ = this->create_wall_timer(
      std::chrono::seconds(1), std::bind(&EnergyCore::timer_callback, this));
  }

private:
  void timer_callback()
  {
    auto message = energy_msgs::msg::PowerState();
    // TODO: Read actual values from sensors
    message.battery_voltage = 12.5;
    message.battery_current = 1.2;
    message.battery_soc = 0.8;
    message.solar_panel_voltage = 18.2;
    message.solar_panel_current = 2.1;
    publisher_->publish(message);
  }
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<energy_msgs::msg::PowerState>::SharedPtr publisher_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<EnergyCore>());
  rclcpp::shutdown();
  return 0;
}
