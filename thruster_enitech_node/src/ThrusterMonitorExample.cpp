#include "rclcpp/rclcpp.hpp"
#include "thruster_enitech_node/ThrusterMonitor.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ThrusterMonitor>();
    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}
