#include "ros2-lifecycle/normal_listener.hpp"

namespace normal
{
    listener::listener(const std::string & node_name)
    :   Node(node_name)
    {
        sub_talker_ = this->create_subscription<std_msgs::msg::String>("lifecycle_talker", 1, [this](const std_msgs::msg::String::ConstSharedPtr msg){this->talker_callback(msg);});
        sub_notification_ = this->create_subscription<lifecycle_msgs::msg::TransitionEvent>("lc_talker/transition_event", 10, [this](const lifecycle_msgs::msg::TransitionEvent::ConstSharedPtr msg){this->notification_callback(msg);});
    }

    listener::~listener()
    {
    }

    void listener::talker_callback(const std_msgs::msg::String::ConstSharedPtr msg)
    {
        RCLCPP_INFO_STREAM(
            this->get_logger(),
            " (talker data) normal_listener received: "
            << msg->data
        );
    }

    void listener::notification_callback(const lifecycle_msgs::msg::TransitionEvent::ConstSharedPtr msg)
    {
        RCLCPP_INFO_STREAM(
            this->get_logger(),
            " (trasition event) normal_listener received: Transition from state "
            << msg->start_state.label.c_str()
            << " to "
            << msg->goal_state.label.c_str()
        );
    }
} // namespace normal