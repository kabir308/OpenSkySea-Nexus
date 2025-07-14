#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "perception_msgs/msg/detected_object_list.hpp"

class VisionHub : public rclcpp::Node
{
public:
  VisionHub()
  : Node("vision_hub")
  {
    publisher_ = this->create_publisher<perception_msgs::msg::DetectedObjectList>("detected_objects", 10);
    subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "laser_scan", 10, std::bind(&VisionHub::topic_callback, this, std::placeholders::_1));
  }

private:
  void topic_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) const
  {
    RCLCPP_INFO(this->get_logger(), "I heard a laser scan");
    // TODO: Process the laser scan and detect objects
    auto detected_objects = perception_msgs::msg::DetectedObjectList();
    publisher_->publish(detected_objects);
  }
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
  rclcpp::Publisher<perception_msgs::msg::DetectedObjectList>::SharedPtr publisher_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<VisionHub>());
  rclcpp::shutdown();
  return 0;
}
