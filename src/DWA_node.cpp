#include <thread>
#include <vector>

#include <galax_navigation/DWA_class.hpp>
#include <galax_navigation/observations_server_class.hpp>

void executor_spin(const std::vector<std::shared_ptr<rclcpp::Node>>& nodes)
{
    rclcpp::executors::MultiThreadedExecutor executor;
    try {
        for (const auto& node : nodes) {
            executor.add_node(node);
        }
        executor.spin();
    } catch (const std::exception & e) {
        for (const auto& node : nodes) {
            RCLCPP_ERROR(node->get_logger(), "Exception: %s", e.what());
        }
    } catch (...) {
        for (const auto& node : nodes) {
            RCLCPP_ERROR(node->get_logger(), "Unknown exception");
        }
    }
    for (const auto& node : nodes) {
        executor.remove_node(node);
    }
    rclcpp::shutdown();
}

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);

    auto dwa_node = std::make_shared<DwaNode>("dwa_node");
    auto observations_server_node = std::make_shared<ObservationsServerNode>("observations_server_node");

    std::vector<std::shared_ptr<rclcpp::Node>> nodes = {dwa_node, observations_server_node};

    std::thread ros_thread(executor_spin, nodes);
    ros_thread.detach();

    // dwa_node->start();
    // observations_server_node->start();

    return 0;
}