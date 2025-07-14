#include "rclcpp/rclcpp.hpp"
#include "multi_channel_msgs/msg/multi_channel_message.hpp"

class MultiChannelStack : public rclcpp::Node
{
public:
  MultiChannelStack()
  : Node("multi_channel_stack")
  {
    publisher_ = this->create_publisher<multi_channel_msgs::msg::MultiChannelMessage>("outgoing_messages", 10);
    subscription_ = this->create_subscription<multi_channel_msgs::msg::MultiChannelMessage>(
      "incoming_messages", 10, std::bind(&MultiChannelStack::topic_callback, this, std::placeholders::_1));
  }

private:
  void topic_callback(const multi_channel_msgs::msg::MultiChannelMessage::SharedPtr msg) const
  {
    RCLCPP_INFO(this->get_logger(), "Received message on channel %s", msg->channel.c_str());
    // TODO: Implement routing logic
    publisher_->publish(*msg);
  }
  rclcpp::Subscription<multi_channel_msgs::msg::MultiChannelMessage>::SharedPtr subscription_;
  rclcpp::Publisher<multi_channel_msgs::msg::MultiChannelMessage>::SharedPtr publisher_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MultiChannelStack>());
  rclcpp::shutdown();
  return 0;
}
