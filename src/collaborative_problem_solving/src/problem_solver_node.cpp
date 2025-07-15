#include "rclcpp/rclcpp.hpp"
#include "collaborative_problem_solving/srv/solve_problem.hpp"

class ProblemSolver : public rclcpp::Node
{
public:
  ProblemSolver()
  : Node("problem_solver")
  {
    service_ = this->create_service<collaborative_problem_solving::srv::SolveProblem>(
      "solve_problem", std::bind(&ProblemSolver::solve_problem, this, std::placeholders::_1, std::placeholders::_2));
  }

private:
  void solve_problem(
    const std::shared_ptr<collaborative_problem_solving::srv::SolveProblem::Request> request,
    std::shared_ptr<collaborative_problem_solving::srv::SolveProblem::Response>      response)
  {
    RCLCPP_INFO(this->get_logger(), "Received problem: %s", request->problem_description.c_str());
    // TODO: Implement collaborative problem solving logic
    response->solution = "Solution found!";
  }

  rclcpp::Service<collaborative_problem_solving::srv::SolveProblem>::SharedPtr service_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ProblemSolver>());
  rclcpp::shutdown();
  return 0;
}
