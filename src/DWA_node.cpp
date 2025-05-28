#include <thread>
#include <vector>

#include <galax_navigation/DWA_class.hpp>
#include "galax_navigation/observations_server_class.hpp"

void executor_spin(const std::vector<std::shared_ptr<rclcpp::Node>>& nodes)
{
    // Create an executur
    rclcpp::executors::MultiThreadedExecutor executor;
    try {
        // Add nodes to the executor
        for (const auto& node : nodes)
        {
            executor.add_node(node);
        }
        // Spin the executor
        executor.spin();
    }
    catch (const std::exception & e)
    {
        for (const auto& node : nodes) {
            RCLCPP_ERROR(node->get_logger(), "Exception: %s", e.what());
        }
    }
    catch (...)
    {
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

    // Create a vector of nodes
    std::vector<std::shared_ptr<rclcpp::Node>> nodes;
    
    // Add the nodes to the vector
    // auto dwa_node = std::make_shared<DwaNode>("dwa_node");
    auto observations_server_node = std::make_shared<ObservationsServerNode>("observations_server_node");

    // nodes.push_back(dwa_node);
    nodes.push_back(observations_server_node);
    
    // Run the spinner of the nodes in a parrallel thread, in case we want to execute a routine in the current main function
    std::thread ros_thread(executor_spin, nodes);
    ros_thread.detach();

    // Start routines if any
    // dwa_node->startRoutine();
    // observations_server_node->startRoutine();

    return 0;
}
