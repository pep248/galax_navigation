#ifndef DWA_NODE_HPP_
#define DWA_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/qos.hpp"
#include "custom_interfaces/msg/dwa.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <galax_navigation/dwa_parameters.hpp>


class DWA_Node : public rclcpp::Node
{
    public:
        DWA_Node();

        void get_params();

        std::shared_ptr<dwa_parameters::ParamListener> dwa_parameters_listener_;
        std::shared_ptr<dwa_parameters::DWA_parameters> dwa_parameters_;

        std::shared_ptr<DWA_parameters_class> dwa_dynamic_parameters_;

    private:
        // Subscribers
        rclcpp::Subscription<custom_interfaces::msg::DWA>::SharedPtr DWA_subscription_;
        void dwa_callback(
            const custom_interfaces::msg::DWA::SharedPtr msg);

        // Publishers
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_publisher_;

        // Timer
        rclcpp::TimerBase::SharedPtr timer_;
        void timer_callback();
};


class DWA_parameters_class
{
    public:
        DWA_parameters();

        // Parameters
        float max_speed;         // v_max
        float min_speed;         // usually 0
        float max_accel;         // a_max
        float max_decel;         // -a_max (if needed)
        float max_yaw_rate;      // omega_max
        float min_yaw_rate;      // -omega_max
        float max_yaw_accel;     // alpha_max
        float max_yaw_decel;     // -alpha_max (if needed)
        float min_turning_radius;// radius
        float heading_coeff;     // alpha (heading)
        float obstacle_coeff;    // beta (obstacle)
        float velocity_coeff;    // gamma (velocity)
        float energy_coeff;      // delta (energy)
        float time_horizon;      // T
        int velocity_samples;    // Ndata

    private:
        // Add any private members or methods if needed
};
#endif  // DWA_NODE_HPP_