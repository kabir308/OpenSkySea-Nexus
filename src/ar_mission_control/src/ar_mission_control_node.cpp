#include "rclcpp/rclcpp.hpp"
#include "ar_mission_control/msg/gesture_command.hpp"
#include "ar_mission_control/msg/voice_command.hpp"
#include "ar_mission_control/msg/visualization_data.hpp"

class ARMissionControl : public rclcpp::Node
{
public:
  ARMissionControl()
  : Node("ar_mission_control")
  {
    visualization_publisher_ = this->create_publisher<ar_mission_control::msg::VisualizationData>("visualization_data", 10);

    gesture_subscription_ = this->create_subscription<ar_mission_control::msg::GestureCommand>(
      "gesture_command", 10, std::bind(&ARMissionControl::gesture_callback, this, std::placeholders::_1));
    voice_subscription_ = this->create_subscription<ar_mission_control::msg::VoiceCommand>(
      "voice_command", 10, std::bind(&ARMissionControl::voice_callback, this, std::placeholders::_1));

    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100), std::bind(&ARMissionControl::publish_visualization_data, this));
  }

private:
  void gesture_callback(const ar_mission_control::msg::GestureCommand::SharedPtr msg) const
  {
    RCLCPP_INFO(this->get_logger(), "Received gesture command: %s", msg->gesture_type.c_str());
    // TODO: Process gesture command
  }

  void voice_callback(const ar_mission_control::msg::VoiceCommand::SharedPtr msg) const
  {
    RCLCPP_INFO(this->get_logger(), "Received voice command: %s", msg->command_text.c_str());
    // TODO: Process voice command
  }

  void publish_visualization_data()
  {
    auto message = ar_mission_control::msg::VisualizationData();
    message.header.stamp = this->now();
    // TODO: Populate with actual visualization data
    message.json_data = "{\"scene\": {}}";
    visualization_publisher_->publish(message);
  }

  rclcpp::Publisher<ar_mission_control::msg::VisualizationData>::SharedPtr visualization_publisher_;
  rclcpp::Subscription<ar_mission_control::msg::GestureCommand>::SharedPtr gesture_subscription_;
  rclcpp::Subscription<ar_mission_control::msg::VoiceCommand>::SharedPtr voice_subscription_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ARMissionControl>());
  rclcpp::shutdown();
  return 0;
}
