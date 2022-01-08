#include "ros2-lifecycle/lifecycle_service_client.hpp"

namespace lifecycle
{
    serviceClient::serviceClient(const std::string & node_name)
    :   rclcpp::Node(node_name)
    {
    }

    serviceClient::~serviceClient()
    {
    }

    void serviceClient::init()
    {
        client_get_state_ = this->create_client<lifecycle_msgs::srv::GetState>(node_get_state_topic);
        client_change_state_ = this->create_client<lifecycle_msgs::srv::ChangeState>(node_change_state_topic);
    }

    unsigned int serviceClient::get_state(std::chrono::seconds time_out)
    {
        auto request = std::make_shared<lifecycle_msgs::srv::GetState::Request>();

        if(!client_get_state_->wait_for_service(time_out))
        {
            RCLCPP_ERROR_STREAM(
                this->get_logger(),
                "Service "
                << client_get_state_->get_service_name()
                << " is not available."
            );
            return lifecycle_msgs::msg::State::PRIMARY_STATE_UNKNOWN;
        }

        // We send the service request for asking the current state of lc_talker
        auto future_result = client_get_state_->async_send_request(request);

        auto future_status = wait_for_result(future_result, time_out);

        if(future_status != std::future_status::ready)
        {
            RCLCPP_ERROR_STREAM(
                this->get_logger(),
                 "Server time out while getting current state for node "
                 << lifecycle_talker_node_name
            );
            return lifecycle_msgs::msg::State::PRIMARY_STATE_UNKNOWN;
        }

        // If succesful
        if(future_result.get())
        {
            RCLCPP_INFO_STREAM(
                this->get_logger(),
                "Node "
                << lifecycle_talker_node_name
                << " has current state "
                << future_result.get()->current_state.label.c_str()
            );
            return future_result.get()->current_state.id;
        }
        else
        {
            RCLCPP_ERROR_STREAM(
                this->get_logger(),
                "Failed to get current state for node "
                << lifecycle_talker_node_name
            );
            return lifecycle_msgs::msg::State::PRIMARY_STATE_UNKNOWN;
        }
    }

    bool serviceClient::change_state(std::uint8_t transition, std::chrono::seconds time_out)
    {
        auto request = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
        request->transition.id = transition;

        if(!client_change_state_->wait_for_service(time_out))
        {
            RCLCPP_ERROR_STREAM(
                this->get_logger(),
                "Service "
                << client_change_state_->get_service_name()
                << " is not available."
            );
            return false;
        }

        auto future_result = client_change_state_->async_send_request(request);

        auto future_status = wait_for_result(future_result, time_out);

        if(future_status != std::future_status::ready)
        {
            RCLCPP_ERROR_STREAM(
                this->get_logger(),
                "Service time out while getting current state for node "
                << lifecycle_talker_node_name
            );
            return false;
        }

        if(future_result.get()->success)
        {
            RCLCPP_INFO_STREAM(
                this->get_logger(),
                "Transition "
                << static_cast<int>(transition)
                << " successfully triggered."
            );
            return true;
        }
        else
        {
            RCLCPP_WARN_STREAM(
                this->get_logger(),
                "Failed to trigger transition "
                << static_cast<unsigned int>(transition)
            );
            return false;
        }
    }

    void callee_script(std::shared_ptr<serviceClient> lc_client)
    {
        rclcpp::WallRate time_between_state_change(0.1);

        // Configure
        {
            if(!rclcpp::ok() || !lc_client->change_state(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE))
                return;
            if(!rclcpp::ok() || !lc_client->get_state())
                return;
        }

        // Activate
        {
            time_between_state_change.sleep();
            if(!rclcpp::ok() || !lc_client->change_state(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE))
                return;
            if(!rclcpp::ok() || !lc_client->get_state())
                return;
        }

        // Deactivate
        {
            time_between_state_change.sleep();
            if(!rclcpp::ok() || !lc_client->change_state(lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE))
                return;
            if(!rclcpp::ok() || !lc_client->get_state())
                return;
        }

        // Activate again
        {
            if(!rclcpp::ok() || !lc_client->change_state(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE))
                return;
            if(!rclcpp::ok() || !lc_client->get_state())
                return;
        }

        // Deactivate again
        {
            time_between_state_change.sleep();
            if(!rclcpp::ok() || !lc_client->change_state(lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE))
                return;
            if(!rclcpp::ok() || !lc_client->get_state())
                return;
        }

        // Clean up
        {
            time_between_state_change.sleep();
            if(!rclcpp::ok() || !lc_client->change_state(lifecycle_msgs::msg::Transition::TRANSITION_CLEANUP))
                return;
            if(!rclcpp::ok() || !lc_client->get_state())
                return;
        }

        // Shutdown
        {
            time_between_state_change.sleep();
            if(!rclcpp::ok() || !lc_client->change_state(lifecycle_msgs::msg::Transition::TRANSITION_UNCONFIGURED_SHUTDOWN))
                return;
            if(!rclcpp::ok() || !lc_client->get_state())
                return;
        }
    }
} // namespace lifecycle