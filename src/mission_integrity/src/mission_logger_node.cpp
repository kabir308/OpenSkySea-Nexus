#include "rclcpp/rclcpp.hpp"
#include "mission_integrity/msg/mission_log.hpp"

class MissionLogger : public rclcpp::Node
{
public:
  MissionLogger()
  : Node("mission_logger")
  {
    publisher_ = this->create_publisher<mission_integrity::msg::MissionLog>("mission_logs", 10);
    subscription_ = this->create_subscription<std_msgs::msg::String>(
      "log_messages", 10, std::bind(&MissionLogger::topic_callback, this, std::placeholders::_1));
  }

private:
  void topic_callback(const std_msgs::msg::String::SharedPtr msg)
  {
    auto log_message = mission_integrity::msg::MissionLog();
    log_message.header.stamp = this->now();
    log_message.mission_id = "test_mission";
    log_message.message = msg->data;
    // TODO: Implement actual hashing
    log_message.previous_hash = "0000000000000000000000000000000000000000000000000000000000000000";
    publisher_->publish(log_message);
  }
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
  rclcpp::Publisher<mission_integrity::msg::MissionLog>::SharedPtr publisher_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MissionLogger>());
  rclcpp::shutdown();
  return 0;
}
