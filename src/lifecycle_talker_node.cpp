#include "ros2-lifecycle/lifecycle_talker.hpp"

int main(int argc, char ** argv)
{
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);

    rclcpp::init(argc, argv);

    rclcpp::executors::SingleThreadedExecutor exe;

    auto lc_node = std::make_shared<lifecycle::talker>("lc_talker");

    exe.add_node(lc_node->get_node_base_interface());

    exe.spin();

    rclcpp::shutdown();

    return 0;
}