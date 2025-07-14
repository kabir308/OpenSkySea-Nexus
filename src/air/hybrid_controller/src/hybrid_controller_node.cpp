#include "rclcpp/rclcpp.hpp"
#include "hybrid_msgs/msg/vehicle_state.hpp"

class HybridController : public rclcpp::Node
{
public:
  HybridController()
  : Node("hybrid_controller")
  {
    publisher_ = this->create_publisher<hybrid_msgs::msg::VehicleState>("vehicle_state_out", 10);
    subscription_ = this->create_subscription<hybrid_msgs::msg::VehicleState>(
      "vehicle_state_in", 10, std::bind(&HybridController::topic_callback, this, std::placeholders::_1));
  }

private:
  void topic_callback(const hybrid_msgs::msg::VehicleState::SharedPtr msg) const
  {
    RCLCPP_INFO(this->get_logger(), "I heard a message");
    // TODO: Process the message and publish a new one
    publisher_->publish(*msg);
  }
  rclcpp::Subscription<hybrid_msgs::msg::VehicleState>::SharedPtr subscription_;
  rclcpp::Publisher<hybrid_msgs::msg::VehicleState>::SharedPtr publisher_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<HybridController>());
  rclcpp::shutdown();
  return 0;
}
