#include <simple_perf_test/probe_component.hpp>
#include <simple_perf_test/target_component.hpp>

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::executors::MultiThreadedExecutor executor;
  auto probe = std::make_shared<simple_perf_test::ProbeComponent>();
  auto target = std::make_shared<simple_perf_test::TargetComponent>();
  executor.add_node(probe);
  executor.add_node(target);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}

