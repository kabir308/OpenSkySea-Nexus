#include "rclcpp/rclcpp.hpp"
#include "hybrid_msgs/msg/vehicle_state.hpp"
#include "seeder_boat_simulator/srv/sow_seeds.hpp"

class SeederBoatSimulator : public rclcpp::Node
{
public:
  SeederBoatSimulator()
  : Node("seeder_boat_simulator")
  {
    publisher_ = this->create_publisher<hybrid_msgs::msg::VehicleState>("matrixpilot/vehicle_state_out", 10);
    subscription_ = this->create_subscription<hybrid_msgs::msg::VehicleState>(
      "matrixpilot/vehicle_state_in", 10, std::bind(&SeederBoatSimulator::topic_callback, this, std::placeholders::_1));

    sow_seeds_service_ = this->create_service<seeder_boat_simulator::srv::SowSeeds>(
      "sow_seeds", std::bind(&SeederBoatSimulator::sow_seeds, this, std::placeholders::_1, std::placeholders::_2));

    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100), std::bind(&SeederBoatSimulator::publish_state, this));
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

  void sow_seeds(
    const std::shared_ptr<seeder_boat_simulator::srv::SowSeeds::Request> request,
    std::shared_ptr<seeder_boat_simulator::srv::SowSeeds::Response>      response)
  {
    RCLCPP_INFO(this->get_logger(), "Sowing seeds");
    response->success = true;
  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<hybrid_msgs::msg::VehicleState>::SharedPtr publisher_;
  rclcpp::Subscription<hybrid_msgs::msg::VehicleState>::SharedPtr subscription_;
  rclcpp::Service<seeder_boat_simulator::srv::SowSeeds>::SharedPtr sow_seeds_service_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SeederBoatSimulator>());
  rclcpp::shutdown();
  return 0;
}
