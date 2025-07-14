#include "rclcpp/rclcpp.hpp"
#include "pluginlib/class_loader.hpp"
#include "nexus_communication_protocol/translator_plugin.hpp"

class NcpTranslator : public rclcpp::Node
{
public:
  NcpTranslator()
  : Node("ncp_translator")
  {
    loader_ = std::make_shared<pluginlib::ClassLoader<nexus_communication_protocol::TranslatorPlugin>>(
      "nexus_communication_protocol", "nexus_communication_protocol::TranslatorPlugin");

    try {
      translator_ = loader_->createSharedInstance("nexus_communication_protocol/MavrosTranslator");
      translator_->initialize(this->get_node_base_interface());
    } catch (pluginlib::PluginlibException & ex) {
      RCLCPP_ERROR(this->get_logger(), "Failed to load translator plugin: %s", ex.what());
    }

    subscription_ = this->create_subscription<nexus_communication_protocol::msg::GoToWaypoint>(
      "ncp/go_to_waypoint", 10, std::bind(&NcpTranslator::topic_callback, this, std::placeholders::_1));
  }

private:
  void topic_callback(const nexus_communication_protocol::msg::GoToWaypoint::SharedPtr msg)
  {
    if (translator_) {
      translator_->translate(msg);
    }
  }

  std::shared_ptr<pluginlib::ClassLoader<nexus_communication_protocol::TranslatorPlugin>> loader_;
  std::shared_ptr<nexus_communication_protocol::TranslatorPlugin> translator_;
  rclcpp::Subscription<nexus_communication_protocol::msg::GoToWaypoint>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<NcpTranslator>());
  rclcpp::shutdown();
  return 0;
}
