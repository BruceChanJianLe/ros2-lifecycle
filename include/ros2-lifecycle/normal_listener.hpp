#ifndef NORMAL_LISTENER_HPP_
#define NORMAL_LISTENER_HPP_

// STL
#include <string>
#include <memory>
#include <chrono>

// ROS2
#include "rclcpp/rclcpp.hpp"

#include "lifecycle_msgs/msg/transition_event.hpp"
#include "std_msgs/msg/string.hpp"

namespace normal
{
    class listener : public rclcpp::Node
    {
    public:
        explicit listener(const std::string & node_name);
        ~listener();

    private:
        std::shared_ptr<rclcpp::Subscription<std_msgs::msg::String>> sub_talker_;
        std::shared_ptr<rclcpp::Subscription<lifecycle_msgs::msg::TransitionEvent>> sub_notification_;

        void talker_callback(const std_msgs::msg::String::ConstSharedPtr msg);
        void notification_callback(const lifecycle_msgs::msg::TransitionEvent::ConstSharedPtr msg);
    };

} // namespace normal

#endif // NORMAL_LISTENER_HPP_