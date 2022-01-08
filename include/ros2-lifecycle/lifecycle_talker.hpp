#ifndef LIFECYCLE_TALKER_HPP_
#define LIFECYCLE_TALKER_HPP_

// STL
#include <string>
#include <memory>
#include <chrono>

// ROS2
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "std_msgs/msg/string.hpp"

namespace lifecycle
{
    class talker : public rclcpp_lifecycle::LifecycleNode
    {
    public:
        explicit talker(const std::string & node_name, bool intra_process_comms = false);
        ~talker();

        void publish();

    protected:
        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_configure(const rclcpp_lifecycle::State &) override;
        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_activate(const rclcpp_lifecycle::State &) override;
        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override;
        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State &) override;
        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_shutdown(const rclcpp_lifecycle::State &) override;

    private:
        // We hold an instance of a lifecycle publisher. This lifecycle publisher
        // can be activated or deactivated regarding on which state the lifecycle node
        // is in.
        // By default, a lifecycle publisher is inactive by creation and has to be
        // activated to publish messages into the ROS world.
        std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::String>> pub_;

        // We hold an instance of a timer which periodically triggers the publish function.
        // As for the beta version, this is a regular timer. In a future version, a
        // lifecycle timer will be created which obeys the same lifecycle management as the
        // lifecycle publisher.
        std::shared_ptr<rclcpp::TimerBase> timer_;

        size_t count_;
    };

} // namespace lifecycle

#endif // LIFECYCLE_TALKER_HPP_