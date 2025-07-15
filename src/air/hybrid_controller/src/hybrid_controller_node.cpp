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
    betaflight_publisher_ = this->create_publisher<hybrid_msgs::msg::VehicleState>("betaflight/vehicle_state_in", 10);
    matrixpilot_publisher_ = this->create_publisher<hybrid_msgs::msg::VehicleState>("matrixpilot/vehicle_state_in", 10);
    sea_publisher_ = this->create_publisher<hybrid_msgs::msg::VehicleState>("sea/vehicle_state_in", 10);
    sub_publisher_ = this->create_publisher<hybrid_msgs::msg::VehicleState>("sub/vehicle_state_in", 10);

    px4_subscription_ = this->create_subscription<hybrid_msgs::msg::VehicleState>(
      "px4/vehicle_state_out", 10, std::bind(&HybridController::px4_topic_callback, this, std::placeholders::_1));
    ardupilot_subscription_ = this->create_subscription<hybrid_msgs::msg::VehicleState>(
      "ardupilot/vehicle_state_out", 10, std::bind(&HybridController::ardupilot_topic_callback, this, std::placeholders::_1));
    betaflight_subscription_ = this->create_subscription<hybrid_msgs::msg::VehicleState>(
      "betaflight/vehicle_state_out", 10, std::bind(&HybridController::betaflight_topic_callback, this, std::placeholders::_1));
    matrixpilot_subscription_ = this->create_subscription<hybrid_msgs::msg::VehicleState>(
      "matrixpilot/vehicle_state_out", 10, std::bind(&HybridController::matrixpilot_topic_callback, this, std::placeholders::_1));
    sea_subscription_ = this->create_subscription<hybrid_msgs::msg::VehicleState>(
      "sea/vehicle_state_out", 10, std::bind(&HybridController::sea_topic_callback, this, std::placeholders::_1));
    sub_subscription_ = this->create_subscription<hybrid_msgs::msg::VehicleState>(
      "sub/vehicle_state_out", 10, std::bind(&HybridController::sub_topic_callback, this, std::placeholders::_1));

    control_mode_service_ = this->create_service<hybrid_msgs::srv::SetControlMode>(
      "set_air_control_mode", std::bind(&HybridController::set_air_control_mode, this, std::placeholders::_1, std::placeholders::_2));
  }

private:
  void px4_topic_callback(const hybrid_msgs::msg::VehicleState::SharedPtr msg) const
  {
    if (air_control_mode_ == "px4") {
      RCLCPP_INFO(this->get_logger(), "Received message from PX4");
      // Forward the message to other domains
      ardupilot_publisher_->publish(*msg);
      betaflight_publisher_->publish(*msg);
      matrixpilot_publisher_->publish(*msg);
      sea_publisher_->publish(*msg);
      sub_publisher_->publish(*msg);
    }
  }

  void ardupilot_topic_callback(const hybrid_msgs::msg::VehicleState::SharedPtr msg) const
  {
    if (air_control_mode_ == "ardupilot") {
      RCLCPP_INFO(this->get_logger(), "Received message from ArduPilot");
      // Forward the message to other domains
      px4_publisher_->publish(*msg);
      betaflight_publisher_->publish(*msg);
      matrixpilot_publisher_->publish(*msg);
      sea_publisher_->publish(*msg);
      sub_publisher_->publish(*msg);
    }
  }

  void betaflight_topic_callback(const hybrid_msgs::msg::VehicleState::SharedPtr msg) const
  {
    if (air_control_mode_ == "betaflight") {
      RCLCPP_INFO(this->get_logger(), "Received message from Betaflight");
      // Forward the message to other domains
      px4_publisher_->publish(*msg);
      ardupilot_publisher_->publish(*msg);
      matrixpilot_publisher_->publish(*msg);
      sea_publisher_->publish(*msg);
      sub_publisher_->publish(*msg);
    }
  }

  void matrixpilot_topic_callback(const hybrid_msgs::msg::VehicleState::SharedPtr msg) const
  {
    RCLCPP_INFO(this->get_logger(), "Received message from MatrixPilot");
    // Forward the message to other domains
    px4_publisher_->publish(*msg);
    ardupilot_publisher_->publish(*msg);
    betaflight_publisher_->publish(*msg);
    sea_publisher_->publish(*msg);
    sub_publisher_->publish(*msg);
  }

  void sea_topic_callback(const hybrid_msgs::msg::VehicleState::SharedPtr msg) const
  {
    RCLCPP_INFO(this->get_logger(), "Received message from sea domain");
    // Forward the message to air and sub domains
    if (air_control_mode_ == "px4") {
      px4_publisher_->publish(*msg);
    } else if (air_control_mode_ == "ardupilot") {
      ardupilot_publisher_->publish(*msg);
    } else {
      betaflight_publisher_->publish(*msg);
    }
    sub_publisher_->publish(*msg);
  }

  void sub_topic_callback(const hybrid_msgs::msg::VehicleState::SharedPtr msg) const
  {
    RCLCPP_INFO(this->get_logger(), "Received message from sub domain");
    // Forward the message to air and sea domains
    if (air_control_mode_ == "px4") {
      px4_publisher_->publish(*msg);
    } else if (air_control_mode_ == "ardupilot") {
      ardupilot_publisher_->publish(*msg);
    } else {
      betaflight_publisher_->publish(*msg);
    }
    sea_publisher_->publish(*msg);
  }

  void set_air_control_mode(
    const std::shared_ptr<hybrid_msgs::srv::SetControlMode::Request> request,
    std::shared_ptr<hybrid_msgs::srv::SetControlMode::Response>      response)
  {
    if (request->mode == "px4" || request->mode == "ardupilot" || request->mode == "betaflight") {
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
  rclcpp::Publisher<hybrid_msgs::msg::VehicleState>::SharedPtr betaflight_publisher_;
  rclcpp::Publisher<hybrid_msgs::msg::VehicleState>::SharedPtr matrixpilot_publisher_;
  rclcpp::Publisher<hybrid_msgs::msg::VehicleState>::SharedPtr sea_publisher_;
  rclcpp::Publisher<hybrid_msgs::msg::VehicleState>::SharedPtr sub_publisher_;
  rclcpp::Subscription<hybrid_msgs::msg::VehicleState>::SharedPtr px4_subscription_;
  rclcpp::Subscription<hybrid_msgs::msg::VehicleState>::SharedPtr ardupilot_subscription_;
  rclcpp::Subscription<hybrid_msgs::msg::VehicleState>::SharedPtr betaflight_subscription_;
  rclcpp::Subscription<hybrid_msgs::msg::VehicleState>::SharedPtr matrixpilot_subscription_;
  rclcpp::Subscription<hybrid_msgs::msg::VehicleState>::SharedPtr sea_subscription_;
  rclcpp::Subscription<hybrid_msgs::msg::VehicleState>::SharedPtr sub_subscription_;
  rclcpp::Service<hybrid_msgs::srv::SetControlMode>::SharedPtr control_mode_service_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<HybridController>());
  rclcpp::shutdown();
  return 0;
}
