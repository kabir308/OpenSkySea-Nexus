#include "rclcpp/rclcpp.hpp"
#include "marketplace/srv/list_modules.hpp"

class Marketplace : public rclcpp::Node
{
public:
  Marketplace()
  : Node("marketplace")
  {
    service_ = this->create_service<marketplace::srv::ListModules>(
      "list_modules", std::bind(&Marketplace::list_modules, this, std::placeholders::_1, std::placeholders::_2));
  }

private:
  void list_modules(
    const std::shared_ptr<marketplace::srv::ListModules::Request> request,
    std::shared_ptr<marketplace::srv::ListModules::Response>      response)
  {
    RCLCPP_INFO(this->get_logger(), "Listing modules");
    // TODO: Implement actual module listing
    response->modules.push_back("Module A");
    response->modules.push_back("Module B");
  }

  rclcpp::Service<marketplace::srv::ListModules>::SharedPtr service_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Marketplace>());
  rclcpp::shutdown();
  return 0;
}
