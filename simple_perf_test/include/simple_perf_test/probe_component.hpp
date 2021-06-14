#ifndef SIMPLE_PERF_TEST_PROBE_COMPONENT_H_
#define SIMPLE_PERF_TEST_PROBE_COMPONENT_H_

#include <rclcpp/rclcpp.hpp>
#include <simple_perf_test_msgs/msg/probe1m.hpp>

using ProbeMsg = simple_perf_test_msgs::msg::Probe1m;

namespace simple_perf_test
{

class ProbeComponent : public rclcpp::Node
{
public:
    ProbeComponent(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
    virtual ~ProbeComponent();

protected:
    void onTimer();

private:
    rclcpp::Publisher<ProbeMsg>::SharedPtr m_pub;
    rclcpp::TimerBase::SharedPtr m_timer;

    // This sequence number is incremented each time a probe is sent
    uint64_t m_seq = 0;

    // The current payload size being tested
    int m_payloadSize;
};

} // end namespace simple_perf_test

#endif /* end #define SIMPLE_PERF_TEST_PROBE_COMPONENT_H_ */
