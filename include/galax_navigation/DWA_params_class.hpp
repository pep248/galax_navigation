#ifndef DWA_PARAMS_CLASS_HPP_
#define DWA_PARAMS_CLASS_HPP_

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/qos.hpp"

#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"
#include <geometry_msgs/msg/transform_stamped.hpp>

#include "nav_msgs/msg/path.hpp"

#include <memory>

class DwaParametersClass
{
    public:
        float alpha = 0.0;         // heading coefficient
        float beta = 0.0;          // obstacle coefficient
        float delta = 0.0;         // energy coefficient
        float gamma = 1.0;         // velocity coefficient
        float time_horizon = 1.0;  // time horizon
        int n_samples = 3;        // number of velocity samples
};

class RobotParametersClass
{
    public:
        // 10 sectors, all initialized to 0.0 (good practice to initialise the vectors if size is known)
        RobotParametersClass() : lidar_ranges(10, 0.0f) {} 

        // Constant parameters
        float mass;
        float inertia;
        float max_speed;    // v_max
        float max_accel;    // a_max
        float max_omega;    // omega_max
        float max_alpha;    // alpha_max
        
        // Dynamic parameters
        geometry_msgs::msg::Pose2D robot_pose;
        geometry_msgs::msg::Twist robot_velocity;

        // Goal state
        geometry_msgs::msg::Pose2D goal_pose;

        // Lidar data
        std::vector<float> lidar_ranges;

        // Path data
        std::vector<nav_msgs::msg::Path> path;
};


#endif  // DWA_PARAMS_CLASS_HPP_