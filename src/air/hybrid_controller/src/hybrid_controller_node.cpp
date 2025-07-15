#include "rclcpp/rclcpp.hpp"
#include "hybrid_msgs/msg/vehicle_state.hpp"
#include "hybrid_msgs/srv/set_control_mode.hpp"

class HybridController : public rclcpp::Node
{
public:
  HybridController()
  : Node("hybrid_controller"),
    air_control_mode_("px4")
  {
    px4_publisher_ = this->create_publisher<hybrid_msgs::msg::VehicleState>("px4/vehicle_state_in", 10);
    ardupilot_publisher_ = this->create_publisher<hybrid_msgs::msg::VehicleState>("ardupilot/vehicle_state_in", 10);
    sea_publisher_ = this->create_publisher<hybrid_msgs::msg::VehicleState>("sea/vehicle_state_in", 10);

    px4_subscription_ = this->create_subscription<hybrid_msgs::msg::VehicleState>(
      "px4/vehicle_state_out", 10, std::bind(&HybridController::px4_topic_callback, this, std::placeholders::_1));
    ardupilot_subscription_ = this->create_subscription<hybrid_msgs::msg::VehicleState>(
      "ardupilot/vehicle_state_out", 10, std::bind(&HybridController::ardupilot_topic_callback, this, std::placeholders::_1));
    sea_subscription_ = this->create_subscription<hybrid_msgs::msg::VehicleState>(
      "sea/vehicle_state_out", 10, std::bind(&HybridController::sea_topic_callback, this, std::placeholders::_1));

    control_mode_service_ = this->create_service<hybrid_msgs::srv::SetControlMode>(
      "set_air_control_mode", std::bind(&HybridController::set_air_control_mode, this, std::placeholders::_1, std::placeholders::_2));
  }

private:
  void px4_topic_callback(const hybrid_msgs::msg::VehicleState::SharedPtr msg) const
  {
    if (air_control_mode_ == "px4") {
      RCLCPP_INFO(this->get_logger(), "Received message from PX4");
      // Forward the message to ArduPilot and sea domain
      ardupilot_publisher_->publish(*msg);
      sea_publisher_->publish(*msg);
    }
  }

  void ardupilot_topic_callback(const hybrid_msgs::msg::VehicleState::SharedPtr msg) const
  {
    if (air_control_mode_ == "ardupilot") {
      RCLCPP_INFO(this->get_logger(), "Received message from ArduPilot");
      // Forward the message to PX4 and sea domain
      px4_publisher_->publish(*msg);
      sea_publisher_->publish(*msg);
    }
  }

  void sea_topic_callback(const hybrid_msgs::msg::VehicleState::SharedPtr msg) const
  {
    RCLCPP_INFO(this->get_logger(), "Received message from sea domain");
    // Forward the message to the active air controller
    if (air_control_mode_ == "px4") {
      px4_publisher_->publish(*msg);
    } else {
      ardupilot_publisher_->publish(*msg);
    }
  }

  void set_air_control_mode(
    const std::shared_ptr<hybrid_msgs::srv::SetControlMode::Request> request,
    std::shared_ptr<hybrid_msgs::srv::SetControlMode::Response>      response)
  {
    if (request->mode == "px4" || request->mode == "ardupilot") {
      air_control_mode_ = request->mode;
      response->success = true;
      RCLCPP_INFO(this->get_logger(), "Air control mode set to %s", air_control_mode_.c_str());
    } else {
      response->success = false;
      RCLCPP_ERROR(this->get_logger(), "Invalid air control mode: %s", request->mode.c_str());
    }
  }

  std::string air_control_mode_;
  rclcpp::Publisher<hybrid_msgs::msg::VehicleState>::SharedPtr px4_publisher_;
  rclcpp::Publisher<hybrid_msgs::msg::VehicleState>::SharedPtr ardupilot_publisher_;
  rclcpp::Publisher<hybrid_msgs::msg::VehicleState>::SharedPtr sea_publisher_;
  rclcpp::Subscription<hybrid_msgs::msg::VehicleState>::SharedPtr px4_subscription_;
  rclcpp::Subscription<hybrid_msgs::msg::VehicleState>::SharedPtr ardupilot_subscription_;
  rclcpp::Subscription<hybrid_msgs::msg::VehicleState>::SharedPtr sea_subscription_;
  rclcpp::Service<hybrid_msgs::srv::SetControlMode>::SharedPtr control_mode_service_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<HybridController>());
  rclcpp::shutdown();
  return 0;
}
