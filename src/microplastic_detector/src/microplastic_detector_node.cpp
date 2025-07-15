#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

class MicroplasticDetector : public rclcpp::Node
{
public:
  MicroplasticDetector()
  : Node("microplastic_detector")
  {
    subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
      "image", 10, std::bind(&MicroplasticDetector::topic_callback, this, std::placeholders::_1));
  }

private:
  void topic_callback(const sensor_msgs::msg::Image::SharedPtr msg) const
  {
    RCLCPP_INFO(this->get_logger(), "Received image");
    // TODO: Implement microplastic detection logic
  }

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MicroplasticDetector>());
  rclcpp::shutdown();
  return 0;
}
