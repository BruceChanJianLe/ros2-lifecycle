#include "ros2-lifecycle/lifecycle_service_client.hpp"

int main(int argc, char ** argv)
{
    // force flush of the stdout buffer.
    // this ensures a correct sync of all prints
    // even when executed simultaneously within the launch file.
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);

    rclcpp::init(argc, argv);

    auto lc_client = std::make_shared<lifecycle::serviceClient>("lc_client");
    lc_client->init();

    rclcpp::executors::SingleThreadedExecutor exe;
    exe.add_node(lc_client);

    std::shared_future<void> script = std::async(std::launch::async, [lc_client](){lifecycle::callee_script(lc_client);});

    exe.spin_until_future_complete(script);

    rclcpp::shutdown();

    return 0;
}