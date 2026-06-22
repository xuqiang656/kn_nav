#include "art_planner_ros/planner_ros.h"



int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>("art_planner");
  art_planner::PlannerRos planner(node);

  const int n_threads = node->get_parameter_or<int>("planner.n_threads", 1);
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), n_threads);
  executor.add_node(node);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}
