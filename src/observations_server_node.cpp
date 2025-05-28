#include <thread>
#include <vector>

// #include <galax_navigation/DWA_class.hpp>
#include "galax_navigation/observations_server_class.hpp"

void executor_spin(std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> executor)
{
    executor->spin();
}

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);

    // Create a MultiThreadedExecutor to handle multiple nodes
    auto executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();

    // Create a vector of nodes
    std::vector<std::shared_ptr<rclcpp::Node>> nodes;

    // Create and add nodes to the executor
    //nodes.push_back(std::make_shared<DwaNode>("dwa_node"));
    nodes.push_back(std::make_shared<ObservationsServerNode>("observations_server_node"));
    for (auto & node : nodes)
    {
        executor->add_node(node);
    }
    
    // Run the spinner of the nodes in a parrallel thread, in case we want to execute a routine in the current main function
    std::thread ros_thread(executor_spin, executor);
    // Optionally, start routines here
    // dwa_node->startRoutine();
    // observations_server_node->startRoutine();

    // Wait for the executor thread to finish (blocks until shutdown)
    ros_thread.join();

    rclcpp::shutdown();
    return 0;
}
