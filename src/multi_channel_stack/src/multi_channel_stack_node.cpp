#include "rclcpp/rclcpp.hpp"
#include "multi_channel_msgs/msg/multi_channel_message.hpp"
#include <map>

class MultiChannelStack : public rclcpp::Node
{
public:
  MultiChannelStack()
  : Node("multi_channel_stack")
  {
    this->declare_parameter<std::vector<std::string>>("channels", std::vector<std::string>());
    this->get_parameter("channels", channels_);

    for (const auto & channel : channels_) {
      publishers_[channel] = this->create_publisher<multi_channel_msgs::msg::MultiChannelMessage>(channel + "/outgoing", 10);
      subscriptions_[channel] = this->create_subscription<multi_channel_msgs::msg::MultiChannelMessage>(
        channel + "/incoming", 10, std::bind(&MultiChannelStack::topic_callback, this, std::placeholders::_1, channel));
    }

    outgoing_subscription_ = this->create_subscription<multi_channel_msgs::msg::MultiChannelMessage>(
      "outgoing_messages", 10, std::bind(&MultiChannelStack::outgoing_callback, this, std::placeholders::_1));
  }

private:
  void topic_callback(const multi_channel_msgs::msg::MultiChannelMessage::SharedPtr msg, const std::string & channel)
  {
    RCLCPP_INFO(this->get_logger(), "Received message on channel %s", channel.c_str());
    // TODO: Implement routing logic
  }

  void outgoing_callback(const multi_channel_msgs::msg::MultiChannelMessage::SharedPtr msg)
  {
    // Simple failover logic: always use the first available channel
    if (!channels_.empty()) {
      publishers_[channels_[0]]->publish(*msg);
    }
  }

  std::vector<std::string> channels_;
  std::map<std::string, rclcpp::Publisher<multi_channel_msgs::msg::MultiChannelMessage>::SharedPtr> publishers_;
  std::map<std::string, rclcpp::Subscription<multi_channel_msgs::msg::MultiChannelMessage>::SharedPtr> subscriptions_;
  rclcpp::Subscription<multi_channel_msgs::msg::MultiChannelMessage>::SharedPtr outgoing_subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MultiChannelStack>());
  rclcpp::shutdown();
  return 0;
}
