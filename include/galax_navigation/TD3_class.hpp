#ifndef TD3_NODE_HPP_
#define TD3_NODE_HPP_

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

class Observations
{
    public:
        Observations() : closest_distance_sector(10, 0.0f) {} // 10 sectors, all initialized to 0.0

        // 1)  Distance Goal
        float distance_goal;
        // 2)  Distance to next path marker
        float distance_next_marker;
        // 3)  Relative Direction to next path marker
        float relative_direction_next_marker;

        // 4) Linear velocity
        float linear_velocity;
        // 5) Angular velocity
        float angular_velocity;

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
        std::vector<float> closest_distance_sector;
}


#endif  // TD3_NODE_HPP_