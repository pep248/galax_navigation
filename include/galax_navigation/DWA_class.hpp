#ifndef DWA_NODE_HPP_
#define DWA_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/qos.hpp"
#include "custom_interfaces/msg/dwa.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"
#include <galax_navigation/dwa_parameters.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <memory>
#include "nav_msgs/msg/path.hpp"


class DwaNode : public rclcpp::Node
{
    public:
        DwaNode(const std::string & node_name);

        void get_params();

        std::shared_ptr<dwa_parameters::ParamListener> dwa_parameters_listener_;
        std::shared_ptr<dwa_parameters::DWA_parameters> dwa_parameters_;

        std::shared_ptr<DwaParametersClass> dwa_parameters_instance_;
        std::shared_ptr<RobotParametersClass> robot_parameters_instance_;

        bool params_loaded = false;

    private:
        // Subscribers
        rclcpp::Subscription<custom_interfaces::msg::DWA>::SharedPtr DWA_subscription_;
        void dwaCallback(
            const custom_interfaces::msg::DWA::SharedPtr msg);

        // Publishers
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_publisher_;

        // Timers
        rclcpp::TimerBase::SharedPtr initial_check_timer_;
        void initialCheckTimerCallback();

        rclcpp::TimerBase::SharedPtr robot_pose_timer_;
        void robotPoseCallback();

        rclcpp::TimerBase::SharedPtr DWA_timer_;
        void dwaTimerCallback();

        // Transform listener
        std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
        std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

        geometry_msgs::msg::Twist CalculateDWA()
};

class DwaParametersClass
{
    public:
        double alpha = 0.0;         // heading coefficient
        double beta = 0.0;          // obstacle coefficient
        double delta = 0.0;         // energy coefficient
        double gamma = 1.0;         // velocity coefficient
        double time_horizon = 1.0;  // time horizon
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


#endif  // DWA_NODE_HPP_