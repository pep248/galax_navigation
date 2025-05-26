#include "rclcpp/rclcpp.hpp"
#include <galax_navigation/DWA_class.hpp>
#include <galax_navigation/observations_server_class.hpp>
#include <thread>
#include <vector>



int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);

    // auto dwa_node = std::make_shared<DwaNode>("dwa_node");
    auto observations_server_node = std::make_shared<ObservationsServerNode>("observations_server_node");
    rclcpp::executors::MultiThreadedExecutor executor;
    // executor.add_node(dwa_node);
    executor.add_node(observations_server_node);
    executor.spin();


    return 0;
}