#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class Rewards : public rclcpp::Node
{
public:
  Rewards()
  : Node("rewards")
  {
    subscription_ = this->create_subscription<std_msgs::msg::String>(
      "contribution", 10, std::bind(&Rewards::topic_callback, this, std::placeholders::_1));
  }

private:
  void topic_callback(const std_msgs::msg::String::SharedPtr msg)
  {
    RCLCPP_INFO(this->get_logger(), "Received contribution: %s", msg->data.c_str());
    // TODO: Implement reward logic
    RCLCPP_INFO(this->get_logger(), "Awarding 100 tokens to %s", msg->data.c_str());
  }

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Rewards>());
  rclcpp::shutdown();
  return 0;
}
