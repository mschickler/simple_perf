#include "simple_perf_test/probe_component.hpp"

#include <chrono>

using namespace std::placeholders;

#define USE_LOANED_MESSAGE 1

namespace simple_perf_test
{

ProbeComponent::ProbeComponent(const rclcpp::NodeOptions& options)
    : rclcpp::Node("probe", options)
{
    // Get node parameters
    int pubDepth = declare_parameter("pub_depth", 1);
    int pubPeriod = declare_parameter("pub_period", 1.0f);
    m_payloadSize = declare_parameter("payload_size", 1024);

    // Allocate ROS resources
    m_pub = create_publisher<ProbeMsg>(
        "probe", pubDepth);
    m_timer = create_wall_timer(
        std::chrono::duration<double>(pubPeriod),
        std::bind(&ProbeComponent::onTimer, this));

    auto t1 = std::chrono::high_resolution_clock::now().time_since_epoch().count();
    auto t2 = std::chrono::high_resolution_clock::now().time_since_epoch().count();
    RCLCPP_INFO_STREAM(get_logger(), "Timestamp overhead = " << t2 -t1 << " nanoseconds");
}

ProbeComponent::~ProbeComponent()
{
}

void ProbeComponent::onTimer()
{
#ifdef USE_LOANED_MESSAGE
    // Allocate memory by borrowing a message buffer from the middleware.
    // This shared memory buffer can be sent to another process with zero copies.
    auto msg = m_pub->borrow_loaned_message();

    RCLCPP_INFO(get_logger(), "---------------------------- Loaned");

    // Grab the timestamp right before the message is sent
    msg.get().timestamp = std::chrono::high_resolution_clock::now().time_since_epoch().count();

    // Send probe.  Since we are dealing with a loaned message, we have to use move
    // semantics.  After the move, the loaned message has been handed off to the
    // middleware and is no longer valid or useable in this scope.
    m_pub->publish(std::move(msg));
#else
    // Allocate memory to be used for the message.  A unique pointer is used because
    // ownership of the message is handed off to the middleware.  When all of the
    // subscribers are finished processing the message, it is automatically deleted.
    std::unique_ptr<ProbeMsg> msg = std::make_unique<ProbeMsg>();

    RCLCPP_INFO(get_logger(), "----------------------------");

    // Make the message the desired size and update bookkeeping fields
    msg->sequence = m_seq++;

    RCLCPP_INFO(get_logger(), "Sending probe with sequence number %lu", msg->sequence);

    // Use this output to verify message pointer is passed to subscriber when using zero copy
    RCLCPP_INFO(get_logger(), "Sent message has address 0x%lx", reinterpret_cast<unsigned long int>(msg.get()));

    // Grab the timestamp right before the message is sent
    msg->timestamp = std::chrono::high_resolution_clock::now().time_since_epoch().count();

    // Send probe.  Since we are dealing with a unique pointer, we have to use move
    // semantics.  After the move, the unique pointer has been handed off to the
    // middleware and is no longer valid or useable in this scope.
    m_pub->publish(std::move(msg));
#endif
}

} // end namespace simple_perf_test

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
RCLCPP_COMPONENTS_REGISTER_NODE(simple_perf_test::ProbeComponent)

