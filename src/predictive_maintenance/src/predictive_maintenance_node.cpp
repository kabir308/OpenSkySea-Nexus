#include "rclcpp/rclcpp.hpp"
#include "predictive_maintenance/msg/component_state.hpp"

class PredictiveMaintenance : public rclcpp::Node
{
public:
  PredictiveMaintenance()
  : Node("predictive_maintenance")
  {
    publisher_ = this->create_publisher<predictive_maintenance::msg::ComponentState>("component_state", 10);
    timer_ = this->create_wall_timer(
      std::chrono::seconds(1), std::bind(&PredictiveMaintenance::publish_state, this));
  }

private:
  void publish_state()
  {
    auto message = predictive_maintenance::msg::ComponentState();
    message.header.stamp = this->now();
    message.component_name = "battery";
    // TODO: Implement actual health prediction
    message.health = 0.95;
    publisher_->publish(message);
  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<predictive_maintenance::msg::ComponentState>::SharedPtr publisher_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PredictiveMaintenance>());
  rclcpp::shutdown();
  return 0;
}
