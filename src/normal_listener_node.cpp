#include "ros2-lifecycle/normal_listener.hpp"

int main(int argc, char ** argv)
{
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);

    rclcpp::init(argc, argv);

    auto node = std::make_shared<normal::listener>("normal_listener");

    rclcpp::spin(node->get_node_base_interface());

    rclcpp::shutdown();

    return 0;
}