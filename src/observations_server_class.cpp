#include "galax_navigation/observations_server.hpp"

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include "nav_msgs/msg/path.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

#include "custom_interfaces/msg/observations.hpp"

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

ObservationsServerNode::ObservationsServerNode(const std::string & node_name) : Node(node_name)
{

    // Initialize transform listener
    this->tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    this->tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    this->robot_pose_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&ObservationsServerNode::robotPoseCallback, this));

    // 1)  Distance Goal
    this->goal_subscription_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "goal",
        1,
        std::bind(&ObservationsServerNode::goalCallback, this, std::placeholders::_1));

    // 2) Distance to next path marker
    // 3) Relative Direction to next path marker
    this->path_subscription_ = this->create_subscription<navigation_msgs::msg::Path>(
        "path",
        1,
        std::bind(&ObservationsServerNode::pathCallback, this, std::placeholders::_1));

    // 4) Linear velocity
    // 5) Angular velocity
    this->velocity_subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "/differential_controller/odom",
        1,
        std::bind(&ObservationsServerNode::velocityCallback, this, std::placeholders::_1));
}

ObservationsServerNode::robotPoseCallback()
{
    // Get the robot pose from the transform listener
    geometry_msgs::msg::TransformStamped transform;
    try {
        transform = tf_buffer_->lookupTransform("pioneer3dx_odom", "map", tf2::TimePointZero);
        robot_pose.x = transform.transform.translation.x;
        robot_pose.y = transform.transform.translation.y;
        robot_pose.theta = tf2::getYaw(transform.transform.rotation);
    } catch (tf2::TransformException & ex) {
        RCLCPP_WARN(this->get_logger(), "Could not get robot pose: %s", ex.what());
    }
}

void ObservationsServerNode::goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
    this->goal_pose.x = msg->pose.position.x;
    this->goal_pose.y = msg->pose.position.y;
}

void ObservationsServerNode::pathCallback(const navigation_msgs::msg::Path::SharedPtr msg)
{
    this->path = *msg;
    if (path.poses.size() > 0)
    {
        this->path_index = 0;
        this->path_index = this->checkPathIndex(this->path_index, *this->path, 0.3f); // TODO -> add distance threshold as parameter
    }
}

int ObservationsServerNode::checkPathIndex(int current_index, nav_msgs::msg::Path& path_, float distance_threshold)
{
    // Check if the path is empty
    if (path_->poses.empty())
    {
        RCLCPP_ERROR(this->get_logger(), "Path is empty");
        return -1;
    }

    // Get the distance to the next pose
    float distance = this->getMarkerDistance(current_index, path_);

    // Check if the distance is less than the threshold
    // If the distance is less than the threshold, move to the next index
    if (distance < distance_threshold && current_index < path_->poses.size() - 1)
    {
        // Move to the next index
        return this->checkPathIndex(current_index + 1, path, distance_threshold);
    }
    // If the distance is greater than the threshold, stay at the current index
    else if (distance >= distance_threshold && current_index < path_->poses.size() - 1)
    {
        return current_index;
    }
    // If the current index is the last index, check if the distance is less than the threshold
    else if (current_index == path_->poses.size() - 1)
    {
        return this->checkPathIndex(current_index + 1, path_, distance_threshold);
    }
    else
    {
        return current_index;
    }
}

float ObservationsServerNode::getMarkerDistance(int index, nav_msgs::msg::Path& path_)
{
    // Get the current pose of the robot
    geometry_msgs::msg::Pose2D robot_pose = this->robot_pose;

    // Get the next pose in the path
    geometry_msgs::msg::PoseStamped next_pose_stamped = path_->poses[current_index];
    
    // Calculate the distance to the next pose
    float distance = std::sqrt(std::pow(next_pose_stamped.pose.position.x - current_pose.x, 2) +
                               std::pow(next_pose_stamped.pose.position.y - current_pose.y, 2) );
    return distance;
}

float ObservationsServerNode::getMarkerOrientation(int index, nav_msgs::msg::Path& path_)
{
// Get the current pose of the robot
    geometry_msgs::msg::Pose2D robot_pose = this->robot_pose;

    // Get the next pose in the path
    geometry_msgs::msg::PoseStamped next_pose_stamped = path_->poses[current_index];

    // Calculate the orientation to the next pose
    float absolute_orientation = std::atan2(next_pose_stamped.pose.position.y - robot_pose.y,
                                            next_pose_stamped.pose.position.x - robot_pose.x);

    // Calculate the relative orientation
    float relative_orientation = absolute_orientation - robot_pose.theta;

    return relative_orientation;
}

void ObservationsServerNode::velocityCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
    this->robot_velocity = *msg;
}

void ObservationsServerNode::laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
    // Assign the laser scan message to the laser_scan_msg_ variable
    this->laser_scan_msg_ = msg;
}

void ObservationsServerNode::filterLaserRanges(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
    // Filter the laser scan data to get the closest distance in each sector
    int num_sectors = 10; // Number of sectors
    float sector_angle = (msg->angle_max - msg->angle_min) / num_sectors;
    this->observations.closest_distance_sector.resize(num_sectors, std::numeric_limits<float>::max());

    for (size_t i = 0; i < msg->ranges.size(); ++i)
    {
        float angle = msg->angle_min + i * msg->angle_increment;
        int sector_index = static_cast<int>((angle - msg->angle_min) / sector_angle);
        if (sector_index >= 0 && sector_index < num_sectors)
        {
            this->observations.closest_distance_sector[sector_index] = std::min(this->observations.closest_distance_sector[sector_index], msg->ranges[i]);
        }
    }
}
//TODO don't save the laser data in each iteration. Instead save the message pointer and use it in the updateObservations function

void ObservationsServerNode::updateObservations()
{
    // Update current marker index
    this->path_index = this->checkPathIndex(this->path_index, *this->path, 0.3f); // TODO -> add distance threshold as parameter
    // 1)  Distance Goal
    this->observations.distance_goal = this->getMarkerDistance(this->path.poses.size()-1, *this->path);
    // 2)  Distance to next path marker
    this->observations.distance_next_marker = this->getMarkerDistance(this->path_index, *this->path);
    // 3)  Relative Direction to next path marker
    this->observations.relative_direction_next_marker = this->getMarkerOrientation(this->path_index, *this->path);
    // 4) Linear velocity
    this->observations.linear_velocity = this->robot_velocity.linear.x;
    // 5) Angular velocity
    this->observations.angular_velocity = this->robot_velocity.angular.z;
    // 6-15) Closest distance sectors
    this->filterLaserRanges(this->laser_scan_msg_);
}

void ObservationsServerNode::normalizeObservations()
{
    // Normalize the observations
    this->observations.distance_goal /= 10.0f; // TODO: Adjust normalization factor
    this->observations.distance_next_marker /= 10.0f; // TODO: Adjust normalization factor
    this->observations.relative_direction_next_marker /= M_PI; // Normalize to [-1, 1]
    this->observations.linear_velocity /= 10.0f; // TODO: Adjust normalization factor
    this->observations.angular_velocity /= M_PI; // Normalize to [-1, 1]
}

void ObservationsServerNode::observationsPublisherTimerCallback()
{
    this->updateObservations();
    this->observations_publisher_->publish(this->observations);
    this->normalized_observations_publisher_->publish(this->observations);
}



#endif  // PATH_PLANNING_SERVER_HPP_