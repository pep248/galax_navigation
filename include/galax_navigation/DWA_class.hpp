#ifndef DWA_NODE_HPP_
#define DWA_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/qos.hpp"
#include "custom_interfaces/msg/dwa.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"
#include <galax_navigation/dwa_parameters_file.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <memory>
#include "nav_msgs/msg/path.hpp"
#include <galax_navigation/DWA_params_class.hpp>


class DwaNode : public rclcpp::Node
{
    public:
        DwaNode(const std::string & node_name);

        void start();

        bool params_loaded = false;

    private:
        // Parameters
        std::shared_ptr<dwa_parameters_file::ParamListener> dwa_parameters_listener_;
        std::shared_ptr<dwa_parameters_file::Params::DwaParams> dwa_parameters_;
        std::shared_ptr<dwa_parameters_file::Params::RobotConstantParams> robot_parameters_;

        std::shared_ptr<DwaParametersClass> dwa_parameters_instance_;
        std::shared_ptr<RobotParametersClass> robot_parameters_instance_;

        void get_params();


        // Subscribers
        rclcpp::Subscription<custom_interfaces::msg::Dwa>::SharedPtr DWA_subscription_;
        void dwaCallback(
            const custom_interfaces::msg::Dwa::SharedPtr msg);


        // Publishers
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_publisher_;


        // Timers
        rclcpp::TimerBase::SharedPtr initial_check_timer_;
        void initialCheckTimerCallback();

        rclcpp::TimerBase::SharedPtr robot_pose_timer_;
        void robotPoseTimerCallback();

        rclcpp::TimerBase::SharedPtr DWA_timer_;
        void dwaTimerCallback();


        // Transform listener
        std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
        std::shared_ptr<tf2_ros::TransformListener> tf_listener_;


        // DWA Algorithm
        geometry_msgs::msg::Twist calculateDWA();
        float calculateHeadingScore(float x_sim, float y_sim, float theta_sim);
        float calculateObstacleScore(float x_sim, float y_sim);
};


#endif  // DWA_NODE_HPP_