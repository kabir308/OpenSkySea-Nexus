#include "rclcpp/rclcpp.hpp"
#include "collaborative_intelligence/msg/knowledge.hpp"

class KnowledgeSharer : public rclcpp::Node
{
public:
  KnowledgeSharer()
  : Node("knowledge_sharer")
  {
    publisher_ = this->create_publisher<collaborative_intelligence::msg::Knowledge>("knowledge", 10);
    subscription_ = this->create_subscription<collaborative_intelligence::msg::Knowledge>(
      "knowledge", 10, std::bind(&KnowledgeSharer::topic_callback, this, std::placeholders::_1));
  }

private:
  void topic_callback(const collaborative_intelligence::msg::Knowledge::SharedPtr msg) const
  {
    RCLCPP_INFO(this->get_logger(), "Received knowledge of type %s", msg->type.c_str());
    // TODO: Process the received knowledge
  }

  void publish_knowledge()
  {
    auto message = collaborative_intelligence::msg::Knowledge();
    message.header.stamp = this->now();
    message.type = "obstacle_location";
    message.data = "{\"x\": 100, \"y\": 200}";
    publisher_->publish(message);
  }

  rclcpp::Subscription<collaborative_intelligence::msg::Knowledge>::SharedPtr subscription_;
  rclcpp::Publisher<collaborative_intelligence::msg::Knowledge>::SharedPtr publisher_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<KnowledgeSharer>());
  rclcpp::shutdown();
  return 0;
}
