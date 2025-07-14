#include "rclcpp/rclcpp.hpp"
#include "mission_orchestrator/msg/mission.hpp"

class MissionGenerator : public rclcpp::Node
{
public:
  MissionGenerator()
  : Node("mission_generator")
  {
    publisher_ = this->create_publisher<mission_orchestrator::msg::Mission>("generated_mission", 10);
    timer_ = this->create_wall_timer(
      std::chrono::seconds(10), std::bind(&MissionGenerator::timer_callback, this));
  }

private:
  void timer_callback()
  {
    auto mission = mission_orchestrator::msg::Mission();
    // TODO: Implement actual mission generation logic
    mission_orchestrator::msg::Objective objective;
    objective.type = "survey";
    objective.target_position.x = 100.0;
    objective.target_position.y = 200.0;
    mission.objectives.push_back(objective);

    mission_orchestrator::msg::Constraint constraint;
    constraint.type = "max_time";
    constraint.value = 3600.0;
    mission.constraints.push_back(constraint);

    publisher_->publish(mission);
  }
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<mission_orchestrator::msg::Mission>::SharedPtr publisher_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MissionGenerator>());
  rclcpp::shutdown();
  return 0;
}
