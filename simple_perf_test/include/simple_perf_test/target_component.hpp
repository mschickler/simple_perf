#ifndef SIMPLE_PERF_TEST_TARGET_COMPONENT_H_
#define SIMPLE_PERF_TEST_TARGET_COMPONENT_H_

#include <rclcpp/rclcpp.hpp>
#include <simple_perf_test_msgs/msg/probe.hpp>

using ProbeMsg = simple_perf_test_msgs::msg::Probe;

namespace simple_perf_test
{

class TargetComponent : public rclcpp::Node
{
public:
    TargetComponent(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
    ~TargetComponent();

protected:
    void onProbe(const ProbeMsg::SharedPtr msg);

private:
    rclcpp::Subscription<ProbeMsg>::SharedPtr m_sub;
};

} // end namespace simple_perf_test

#endif /* end #define SIMPLE_PERF_TEST_TARGET_COMPONENT_H_ */
