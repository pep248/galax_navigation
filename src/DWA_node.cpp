#include <galax_navigation/DWA_class.hpp>

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto DWA_node = std::make_shared<DWA_Node>();

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(DWA_node);
    executor.spin();

    rclcpp::shutdown();
    return 0;
}