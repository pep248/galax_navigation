#ifndef OBSERVATIONS_SERVER_HPP_
#define OBSERVATIONS_SERVER_HPP_

// ===== Standard libraries =====
#include <memory>

// ===== ROS-specific imports =====
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/qos.hpp"

#include "geometry_msgs/msg/pose2_d.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_listener.h>

// ===== Custom parameters and project-specific headers =====
#include <custom_interfaces/msg/observations.hpp>
#include <galax_navigation/observations_class.hpp>


class ObservationsServerNode : public rclcpp::Node
{
    public:
        ObservationsServerNode(const std::string & node_name);

        void start();

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
        bool pose_received = false;


        // 1)  Distance Goal
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_subscription_;
        void goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
        geometry_msgs::msg::Pose2D goal_pose;
        bool goal_received = false;
        

        // 2)  Distance to next path marker
        // 3)  Relative Direction to next path marker
        rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_subscription_;
        void pathCallback(const nav_msgs::msg::Path::SharedPtr msg);
        bool path_received = false;
        nav_msgs::msg::Path::SharedPtr path_;
        int path_index = 0;
        int checkPathIndex(int current_index, nav_msgs::msg::Path::SharedPtr path_, float distance_threshold = 0.3f); // TODO -> add distance threshold as parameter
        float getMarkerDistance(int index, nav_msgs::msg::Path::SharedPtr path_);
        float getMarkerOrientation(int index, nav_msgs::msg::Path::SharedPtr path_);
        

        // 4) Linear velocity
        // 5) Angular velocity
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr velocity_subscription_;
        void velocityCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
        bool velocity_received;

        
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
        bool laser_received = false;
        sensor_msgs::msg::LaserScan::SharedPtr laser_scan_msg_;
        std::vector<float> filterLaserRanges(const sensor_msgs::msg::LaserScan::SharedPtr msg);


        // Publishers
        rclcpp::Publisher<custom_interfaces::msg::Observations>::SharedPtr observations_publisher_;
        rclcpp::Publisher<custom_interfaces::msg::Observations>::SharedPtr normalized_observations_publisher_;
        rclcpp::TimerBase::SharedPtr observations_publisher_timer_;
        bool all_data_received = false;
        void observationsPublisherTimerCallback();
        void updateObservations();
        void normalizeObservations();

        void publishObservations();
        void publishNormalizedObservations();

};

#endif  // OBSERVATIONS_SERVER_HPP_