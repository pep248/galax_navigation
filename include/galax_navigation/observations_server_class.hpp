#ifndef OBSERVATIONS_SERVER_HPP_
#define OBSERVATIONS_SERVER_HPP_

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include "nav_msgs/msg/path.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

#include "custom_interfaces/msg/observations.hpp"

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

class ObservationsServerNode : public rclcpp::Node
{
    public:
        ObservationsServerNode(const std::string & node_name)
            : Node(node_name),  laser_ranges(10, 0.0f) {};

    private:
        // Observations
        Observations observations;
        Observations normalized_observations;

        // Robot pose
        std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
        std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

        geometry_msgs::msg::Pose2D robot_pose;
        geometry_msgs::msg::Twist robot_velocity;

        rclcpp::TimerBase::SharedPtr robot_pose_timer_;
        void robotPoseCallback();


        // 1)  Distance Goal
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_subscription_;
        void goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
        geometry_msgs::msg::Pose2D goal_pose;
        
        // 2)  Distance to next path marker
        rclcpp::Subscription<navigation_msgs::msg::Path>::SharedPtr path_subscription_;
        void pathCallback(const navigation_msgs::msg::Path::SharedPtr msg);
        navigation_msgs::msg::Path path;
        int path_index = 0;
        int checkPathIndex(int current_index, nav_msgs::msg::Path& path_, float distance_threshold = 0.3f); // TODO -> add distance threshold as parameter
        float getMarkerDistance(int index, nav_msgs::msg::Path& path_);
        // 3)  Relative Direction to next path marker

        // 4) Linear velocity
        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr velocity_subscription_;
        void velocityCallback(const geometry_msgs::msg::Twist::SharedPtr msg);

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
        rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_subscription_;
        void laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
        sensor_msgs::msg::LaserScan::SharedPtr laser_scan_msg_;
        void filterLaserRanges(const sensor_msgs::msg::LaserScan::SharedPtr msg);

        // Publishers
        rclcpp::Publisher<custom_interfaces::msg::Observations>::SharedPtr observations_publisher_;
        rclcpp::Publisher<custom_interfaces::msg::Observations>::SharedPtr normalized_observations_publisher_;
        rclcpp::TimerBase::SharedPtr observations_publisher_timer_;
        void observationsPublisherTimerCallback();



}

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

class Normalized_Observations : public Observations
{
    public:
        Normalized_Observations() : closest_distance_sector(10, 0.0f) {}
}

#endif  // PATH_PLANNING_SERVER_HPP_