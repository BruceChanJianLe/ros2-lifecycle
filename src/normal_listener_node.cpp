#include "ros2-lifecycle/normal_listener.hpp"

int main(int argc, char ** argv)
{
    // force flush of the stdout buffer.
    // this ensures a correct sync of all prints
    // even when executed simultaneously within the launch file.
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);

    rclcpp::init(argc, argv);

    auto node = std::make_shared<normal::listener>("normal_listener");

    rclcpp::spin(node->get_node_base_interface());

    rclcpp::shutdown();

    return 0;
}