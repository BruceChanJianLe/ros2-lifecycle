#include "ros2-lifecycle/lifecycle_talker.hpp"

namespace lifecycle
{
    talker::talker(const std::string & node_name, bool intra_process_comms)
    :   rclcpp_lifecycle::LifecycleNode(node_name, rclcpp::NodeOptions().use_intra_process_comms(intra_process_comms))
    ,   count_(0)
    {
    }

    talker::~talker()
    {
    }

    void talker::publish()
    {
        auto msg = std::make_unique<std_msgs::msg::String>();
        msg->data = "Lifecycle talker published: " + std::to_string(count_++);

        if(!pub_->is_activated())
        {
            RCLCPP_INFO_STREAM(
                this->get_logger(),
                "Lifecycle publisher is currently inactive. Messages are not published."
            );
        }
        else
        {
            RCLCPP_INFO_STREAM(
                this->get_logger(),
                "Lifecycle publisher is currently active. Publishing: ["
                << msg->data
                << "]"
            );
        }

        pub_->publish(std::move(msg));
    }

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn talker::on_configure(const rclcpp_lifecycle::State & state)
    {
        try
        {
            pub_ = this->create_publisher<std_msgs::msg::String>("lifecycle_talker", 1);
            timer_ = this->create_wall_timer(std::chrono::seconds(1), [this](){this->publish();});
        }
        catch(const rclcpp::exceptions::RCLError & e)
        {
            RCLCPP_ERROR_STREAM(
                this->get_logger(),
                __func__
                << "() throw an error: "
                << e.what()
            );

            return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
        }

        RCLCPP_INFO_STREAM(
            this->get_logger(),
            __func__
            << "() is called."
        );

        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn talker::on_activate(const rclcpp_lifecycle::State & state)
    {
        try
        {
            pub_->on_activate();
        }
        catch(const rclcpp::exceptions::RCLError & e)
        {
            RCLCPP_ERROR_STREAM(
                this->get_logger(),
                __func__
                << "() throw an error: "
                << e.what()
            );

            return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
        }

        RCLCPP_INFO_STREAM(
            this->get_logger(),
            __func__
            << "() is called."
        );

        // Emulate we are doing sth important
        std::this_thread::sleep_for(std::chrono::seconds(2));
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn talker::on_deactivate(const rclcpp_lifecycle::State & state)
    {
        try
        {
            pub_->on_deactivate();
        }
        catch(const rclcpp::exceptions::RCLError & e)
        {
            RCLCPP_ERROR_STREAM(
                this->get_logger(),
                __func__
                << "() throw an error: "
                << e.what()
            );

            return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
        }

        RCLCPP_INFO_STREAM(
            this->get_logger(),
            __func__
            << "() is called."
        );

        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn talker::on_cleanup(const rclcpp_lifecycle::State & state)
    {
         try
        {
            timer_.reset();
            pub_.reset();
        }
        catch(const rclcpp::exceptions::RCLError & e)
        {
            RCLCPP_ERROR_STREAM(
                this->get_logger(),
                __func__
                << "() throw an error: "
                << e.what()
            );

            return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
        }

        RCLCPP_INFO_STREAM(
            this->get_logger(),
            __func__
            << "() is called."
        );

        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn talker::on_shutdown(const rclcpp_lifecycle::State & state)
    {
         try
        {
            timer_.reset();
            pub_.reset();
        }
        catch(const rclcpp::exceptions::RCLError & e)
        {
            RCLCPP_ERROR_STREAM(
                this->get_logger(),
                __func__
                << "() throw an error: "
                << e.what()
            );

            return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
        }

        RCLCPP_INFO_STREAM(
            this->get_logger(),
            __func__
            << "() is called from state "
            << state.label().c_str()
        );

        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }
} // namespace lifecycle