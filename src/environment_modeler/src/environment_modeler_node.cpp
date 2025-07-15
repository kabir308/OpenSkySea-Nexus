#include "rclcpp/rclcpp.hpp"
#include "environment_modeler/msg/ocean_current.hpp"
#include "environment_modeler/msg/weather.hpp"
#include "environment_modeler/msg/marine_life.hpp"
#include "std_srvs/srv/trigger.hpp"

class EnvironmentModeler : public rclcpp::Node
{
public:
  EnvironmentModeler()
  : Node("environment_modeler")
  {
    ocean_current_publisher_ = this->create_publisher<environment_modeler::msg::OceanCurrent>("ocean_current", 10);
    weather_publisher_ = this->create_publisher<environment_modeler::msg::Weather>("weather", 10);
    marine_life_publisher_ = this->create_publisher<environment_modeler::msg::MarineLife>("marine_life", 10);

    weather_client_ = this->create_client<std_srvs::srv::Trigger>("get_weather_forecast");

    timer_ = this->create_wall_timer(
      std::chrono::seconds(5), std::bind(&EnvironmentModeler::timer_callback, this));
  }

private:
  void timer_callback()
  {
    // TODO: Implement actual environment modeling
    auto ocean_current = environment_modeler::msg::OceanCurrent();
    ocean_current.velocity.x = 0.5;
    ocean_current_publisher_->publish(ocean_current);

    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
    auto result = weather_client_->async_send_request(request);
    // TODO: Process weather forecast result

    auto marine_life = environment_modeler::msg::MarineLife();
    marine_life.species = "whale";
    marine_life.position.x = 1000.0;
    marine_life.position.y = 2000.0;
    marine_life_publisher_->publish(marine_life);
  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<environment_modeler::msg::OceanCurrent>::SharedPtr ocean_current_publisher_;
  rclcpp::Publisher<environment_modeler::msg::Weather>::SharedPtr weather_publisher_;
  rclcpp::Publisher<environment_modeler::msg::MarineLife>::SharedPtr marine_life_publisher_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr weather_client_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<EnvironmentModeler>());
  rclcpp::shutdown();
  return 0;
}
