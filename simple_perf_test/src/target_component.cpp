#include "simple_perf_test/target_component.hpp"

using namespace std::placeholders;

namespace simple_perf_test
{

TargetComponent::TargetComponent(const rclcpp::NodeOptions& options)
    : rclcpp::Node("target", options)
{
    // Get node parameters
    int subDepth = declare_parameter("sub_depth", 1);

    // Allocate ROS resources
    m_sub = create_subscription<ProbeMsg>(
        "probe", subDepth, std::bind(&TargetComponent::onProbe, this, _1));
}

TargetComponent::~TargetComponent()
{
}

void TargetComponent::onProbe(const ProbeMsg::SharedPtr msg)
{
    // Grab the final timestamp as soon as possible
    uint64_t t2 = std::chrono::high_resolution_clock::now().time_since_epoch().count();

    // Confirm sequence number
    RCLCPP_INFO(get_logger(), "Received probe with sequence number %d", msg->sequence);

    // Verify original message pointer was passed in intra-process case
    RCLCPP_INFO(get_logger(), "Received message has address 0x%lx", reinterpret_cast<unsigned long int>(msg.get()));

    // Output the delay
    RCLCPP_INFO_STREAM(get_logger(), "Latency = " << t2 - msg->timestamp << " nanoseconds");
}

  

} // end namespace simple_perf_test

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
RCLCPP_COMPONENTS_REGISTER_NODE(simple_perf_test::TargetComponent)

