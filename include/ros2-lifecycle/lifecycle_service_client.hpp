#ifndef LIFECYCLE_SERVICE_CLIENT_HPP_
#define LIFECYCLE_SERVICE_CLIENT_HPP_

// STL
#include <string>
#include <memory>
#include <chrono>
#include <thread>
#include <future>

// ROS2
#include <rclcpp/rclcpp.hpp>

#include "lifecycle_msgs/msg/state.hpp"
#include "lifecycle_msgs/msg/transition.hpp"
#include "lifecycle_msgs/srv/change_state.hpp"
#include "lifecycle_msgs/srv/get_state.hpp"

constexpr auto lifecycle_talker_node_name = "lc_talker";
constexpr auto node_get_state_topic = "lc_talker/get_state";
constexpr auto node_change_state_topic = "lc_talker/change_state";


namespace lifecycle
{

    template <typename FutureT, typename WaitTimeT>
    std::future_status wait_for_result(FutureT & future, WaitTimeT time_to_wait)
    {
        auto end = std::chrono::steady_clock::now() + time_to_wait;
        std::chrono::milliseconds wait_period(100);
        std::future_status status = std::future_status::timeout;

        do
        {
            auto now = std::chrono::steady_clock::now();
            auto time_left = end - now;
            if(time_left <= std::chrono::seconds(0))
                break;
            status = future.wait_for((time_left < wait_period) ? time_left : wait_period);
        }while(rclcpp::ok() && status != std::future_status::ready);

        return status;
    }

    class serviceClient : public rclcpp::Node
    {
    public:
        explicit serviceClient(const std::string & node_name);
        ~serviceClient();

        void init();
        unsigned int get_state(std::chrono::seconds time_out = std::chrono::seconds(3));
        bool change_state(std::uint8_t transition, std::chrono::seconds time_out = std::chrono::seconds(3));
    private:
        std::shared_ptr<rclcpp::Client<lifecycle_msgs::srv::GetState>> client_get_state_;
        std::shared_ptr<rclcpp::Client<lifecycle_msgs::srv::ChangeState>> client_change_state_;
    };

    void callee_script(std::shared_ptr<serviceClient> lc_client);

} // namespace lifecycle

#endif // LIFECYCLE_SERVICE_CLIENT_HPP_