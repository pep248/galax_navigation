#ifndef DWA_NODE_HPP_
#define DWA_NODE_HPP_

// ===== Standard libraries =====
#include <memory>

// ===== ROS-specific imports =====
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/qos.hpp"

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/path.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "pcl_conversions/pcl_conversions.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"


// ===== Custom parameters and project-specific headers =====
#include <custom_interfaces/msg/dwa.hpp>
#include <custom_interfaces/msg/observations.hpp>
#include <galax_navigation/dwa_parameters_file.hpp>
#include <galax_navigation/DWA_params_class.hpp>
#include <galax_navigation/observations_class.hpp>



class DwaNode : public rclcpp::Node
{
    public:
        DwaNode(const std::string & node_name);

        void start();

    private:
        // Observations
        // 1)  Distance Goal
        // 2)  Distance to next path marker
        // 3)  Relative Direction to next path marker

        // 4) Linear velocity
        // 5) Angular velocity

        // 6) Closest distance sector 1 (1-12)
        // 7) Closest distance sector 2 (13-24)
        // 8) Closest distance sector 3 (25-36)
        // 9) Closest distance sector 4 (37-48)
        // 10) Closest distance sector 5 (49-60)
        // 11) Closest distance sector 6 (61-72)
        // 12) Closest distance sector 7 (73-84)
        // 13) Closest distance sector 8 (85-96)
        // 14) Closest distance sector 9 (97-108)
        // 15) Closest distance sector 10 (109-120)

        custom_interfaces::msg::Observations::SharedPtr observations_;
        custom_interfaces::msg::Observations::SharedPtr normalized_observations_;


        // Parameters
        std::shared_ptr<dwa_parameters_file::ParamListener> file_parameters_listener_;
        std::shared_ptr<dwa_parameters_file::Params> file_parameters_;

        std::shared_ptr<DwaParametersClass> dwa_parameters_instance_;
        std::shared_ptr<RobotParametersClass> robot_parameters_instance_;

        void get_params();


        // Subscribers
        rclcpp::Subscription<custom_interfaces::msg::Dwa>::SharedPtr dwa_subscription_;
        void updateDwaCallback(
            const custom_interfaces::msg::Dwa::SharedPtr msg);

        rclcpp::Subscription<custom_interfaces::msg::Observations>::SharedPtr observations_subscription_;
        void observationsCallback(
            const custom_interfaces::msg::Observations::SharedPtr msg);
        bool observations_ready_ = false;

        rclcpp::Subscription<custom_interfaces::msg::Observations>::SharedPtr normalized_observations_subscription_;
        void normalizedObservationsCallback(
            const custom_interfaces::msg::Observations::SharedPtr msg);
        bool normalized_observations_ready_ = false;

        rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_subscription_;
        void lidarCallback(
            const sensor_msgs::msg::LaserScan::SharedPtr msg);
        bool lidar_data_ready_ = false;
        sensor_msgs::msg::LaserScan::SharedPtr lidar_data_;


        // Publishers
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_publisher_;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr dwa_map_publisher_;


        // Timers
        rclcpp::TimerBase::SharedPtr dwa_timer_;
        void dwaTimerCallback();
        bool all_data_received = false;


        // DWA Algorithm
        geometry_msgs::msg::Twist calculateDWA();
        float calculateHeadingScore();
        float calculateObstacleScore(float x_sim, float y_sim);
};


#endif  // DWA_NODE_HPP_