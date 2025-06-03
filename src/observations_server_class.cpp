#include "galax_navigation/observations_server_class.hpp"



// Node Constructor
ObservationsServerNode::ObservationsServerNode(const std::string & node_name)
 : Node(node_name)
{
    RCLCPP_INFO(get_logger(), "ObservationsServerNode initialized");
    
    // Initialize transform listener
    this->tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    this->tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    this->robot_pose_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&ObservationsServerNode::robotPoseCallback, this));

    // 1)  Distance Goal
    this->goal_subscription_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/goal_pose",
        1,
        std::bind(&ObservationsServerNode::goalCallback, this, std::placeholders::_1));

    // 2) Distance to next path marker
    // 3) Relative Direction to next path marker
    this->path_subscription_ = this->create_subscription<nav_msgs::msg::Path>(
        "/path",
        1,
        std::bind(&ObservationsServerNode::pathCallback, this, std::placeholders::_1));

    // 4) Linear velocity
    // 5) Angular velocity
    this->velocity_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/diff_cont/odom",
        1,
        std::bind(&ObservationsServerNode::velocityCallback, this, std::placeholders::_1));

    // 6-15) Closest distance sectors
    this->laser_subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan",
        1,
        std::bind(&ObservationsServerNode::laserCallback, this, std::placeholders::_1));

    // Observations publisher
    this->observations_publisher_ = this->create_publisher<custom_interfaces::msg::Observations>(
        "/observations",
        1);
        
    this->normalized_observations_publisher_ = this->create_publisher<custom_interfaces::msg::Observations>(
        "/normalized_observations",
        1);
    this->robot_dwa_publisher_ = this->create_publisher<custom_interfaces::msg::Dwa>(
        "/dwa_parameters",
        1);

    this->observations_publisher_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&ObservationsServerNode::observationsPublisherTimerCallback, this));
}


void ObservationsServerNode::start()
{
    
}


// Robot pose callback
void ObservationsServerNode::robotPoseCallback()
{
    // Get the robot pose from the transform listener
    
    while (!tf_buffer_->canTransform("map", "galax_base_link", rclcpp::Time(0), rclcpp::Duration::from_seconds(1.0))) {
        RCLCPP_INFO(this->get_logger(), "Waiting for transform map -> galax_base_link...");
        rclcpp::sleep_for(std::chrono::milliseconds(1000));
    }
    geometry_msgs::msg::TransformStamped transform;
    transform = tf_buffer_->lookupTransform("map", "galax_base_link", rclcpp::Time(0), rclcpp::Duration::from_seconds(1.0));
    robot_pose.x = transform.transform.translation.x;
    robot_pose.y = transform.transform.translation.y;

    // Convert quaternion to yaw (theta)
    tf2::Quaternion q(
        transform.transform.rotation.x,
        transform.transform.rotation.y,
        transform.transform.rotation.z,
        transform.transform.rotation.w
    );
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    robot_pose.theta = yaw;
    this->pose_received = true;
}


// Goal callback
void ObservationsServerNode::goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
    this->goal_pose.x = msg->pose.position.x;
    this->goal_pose.y = msg->pose.position.y;
    this->goal_received = true;
}


// Path callback
void ObservationsServerNode::pathCallback(const nav_msgs::msg::Path::SharedPtr msg)
{
    this->path_ = msg;
    if (path_->poses.size() > 0)
    {
        this->path_index = 0;
        this->path_index = this->checkPathIndex(this->path_index, this->path_, 0.3f); // TODO -> add distance threshold as parameter
    }
    this->path_received = true;
}


int ObservationsServerNode::checkPathIndex(int current_index, nav_msgs::msg::Path::SharedPtr path_, float distance_threshold)
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
        return this->checkPathIndex(current_index + 1, path_, distance_threshold);
    }
    // If the distance is greater than the threshold, stay at the current index
    else if (distance >= distance_threshold && current_index < path_->poses.size() - 1)
    {
        return current_index;
    }
    // If the current index is the last index, check if the distance is less than the threshold
    else if (current_index == path_->poses.size() - 1)
    {
        //check if the final marker in path is within the distance threshold
        if (distance < distance_threshold)
        {
            // If the distance is less than the threshold update the dwa parameters to stop the robot
            custom_interfaces::msg::Dwa msg;
            msg.alpha = 0.0f;
            msg.beta = 0.0f;
            msg.gamma = 0.0f;
            msg.delta = 1.0f; // Set delta to 1.0 to stop the robot
            this->robot_dwa_publisher_->publish(msg); 
            RCLCPP_INFO(this->get_logger(), "Reached the final marker in path, stopping the robot.");
        }
        else
        {
            // If the distance is greater than the threshold, stay at the current index
            return this->checkPathIndex(current_index + 1, path_, distance_threshold);
        }
    }
    else
    {
        return current_index;
    }
    return current_index; // fallback
}


float ObservationsServerNode::getMarkerDistance(int index, nav_msgs::msg::Path::SharedPtr path_)
{
    // Get the next pose in the path
    geometry_msgs::msg::PoseStamped next_pose_stamped = path_->poses[index];
    
    // Calculate the distance to the next pose
    float distance = std::sqrt(std::pow(next_pose_stamped.pose.position.x - this->robot_pose.x, 2) +
                               std::pow(next_pose_stamped.pose.position.y - this->robot_pose.y, 2) );
    return distance;
}


float ObservationsServerNode::getMarkerOrientation(int index, nav_msgs::msg::Path::SharedPtr path_)
{
// Get the current pose of the robot
    auto& robot_pose = this->robot_pose;

    // Get the next pose in the path
    geometry_msgs::msg::PoseStamped next_pose_stamped = path_->poses[index];

    // Calculate the orientation to the next pose
    float absolute_orientation = std::atan2(next_pose_stamped.pose.position.y - robot_pose.y,
                                            next_pose_stamped.pose.position.x - robot_pose.x);

    // Calculate the relative orientation
    float relative_orientation = absolute_orientation - robot_pose.theta;

    return relative_orientation;
}


// Velocity callback
void ObservationsServerNode::velocityCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    
    this->robot_velocity = msg->twist.twist;
    this->velocity_received = true;
}


// Laser scan callback
void ObservationsServerNode::laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
    // Assign the laser scan message to the laser_scan_msg_ variable
    this->laser_scan_msg_ = msg;
    this->laser_received = true;
}


std::vector<float> ObservationsServerNode::filterLaserRanges(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
    int num_sectors = 10; // Number of sectors
    float sector_angle = (msg->angle_max - msg->angle_min) / num_sectors;
    std::vector<float> closest_distance_sector(num_sectors, std::numeric_limits<float>::max());

    for (size_t i = 0; i < msg->ranges.size(); ++i)
    {
        float angle = msg->angle_min + i * msg->angle_increment;
        int sector_index = static_cast<int>((angle - msg->angle_min) / sector_angle);
        if (sector_index >= 0 && sector_index < num_sectors)
        {
            closest_distance_sector[sector_index] = std::min(closest_distance_sector[sector_index], msg->ranges[i]);
        }
    }
    return closest_distance_sector;
}


// Observations publisher timer callback
void ObservationsServerNode::observationsPublisherTimerCallback()
{
    if (this->all_data_received == true)
    {
        this->updateObservations();
        this->normalizeObservations();
        this->publishObservations();
        this->publishNormalizedObservations();
    }
    else
    {
        if (this->pose_received && this->goal_received && this->path_received && this->velocity_received && this->laser_received)
        {
            RCLCPP_INFO(this->get_logger(), "All data received, starting observations update.");
            this->all_data_received = true;
        }
        else
        {
            if (!this->pose_received)
            {
                RCLCPP_WARN(this->get_logger(), "Pose data not received yet.");
            }
            if (!this->goal_received)
            {
                RCLCPP_WARN(this->get_logger(), "Goal data not received yet.");
            }
            if (!this->path_received)
            {
                RCLCPP_WARN(this->get_logger(), "Path data not received yet.");
            }
            if (!this->velocity_received)
            {
                RCLCPP_WARN(this->get_logger(), "Velocity data not received yet.");
            }
            if (!this->laser_received)
            {
                RCLCPP_WARN(this->get_logger(), "Laser scan data not received yet.");
            }
        }
    }
}


void ObservationsServerNode::updateObservations()
{
    // Update current marker index
    this->path_index = this->checkPathIndex(this->path_index, this->path_, 0.3f); // TODO -> add distance threshold as parameter
    // 1)  Distance Goal
    this->observations.distance_goal = this->getMarkerDistance(this->path_->poses.size()-1, this->path_);
    // 2)  Distance to next path marker
    this->observations.distance_next_marker = this->getMarkerDistance(this->path_index, this->path_);
    // 3)  Relative Direction to next path marker
    this->observations.relative_direction_next_marker = this->getMarkerOrientation(this->path_index, this->path_);
    // 4) Linear velocity
    this->observations.linear_velocity = this->robot_velocity.linear.x;
    // 5) Angular velocity
    this->observations.angular_velocity = this->robot_velocity.angular.z;
    // 6-15) Closest distance sectors
    this->observations.closest_distance_sector = this->filterLaserRanges(this->laser_scan_msg_);
}


void ObservationsServerNode::normalizeObservations()
{
    // Example values, replace with your actual robot/environment parameters
    float goal_distance_from_origin = std::sqrt(std::pow(this->goal_pose.x, 2) + std::pow(this->goal_pose.y, 2));
    float marker_max_separation = 3.0f; // Set this to your actual value
    float v_max = 1.0f;                 // Set this to your actual value
    float omega_max = 1.2f;             // Set this to your actual value
    float lidar_max_range = 30.0f;      // Set this to your actual value

    // 1) Distance to goal [0,1]
    if (goal_distance_from_origin > 0.0f)
        this->normalized_observations.distance_goal = std::min(this->observations.distance_goal / goal_distance_from_origin, 1.0f);
    else
        this->normalized_observations.distance_goal = 0.0f;

    // 2) Distance to next path marker [0,1]
    this->normalized_observations.distance_next_marker = std::min(this->observations.distance_next_marker / marker_max_separation, 1.0f);

    // 3) Relative direction to next marker [-1,1]
    this->normalized_observations.relative_direction_next_marker = this->observations.relative_direction_next_marker / static_cast<float>(M_PI);

    // 4) Linear velocity [0,1]
    this->normalized_observations.linear_velocity = std::min(this->observations.linear_velocity / v_max, 1.0f);

    // 5) Angular velocity [-1, 1]
    this->normalized_observations.angular_velocity =
        std::max(std::min(this->observations.angular_velocity / omega_max, 1.0f), -1.0f);

    // 6-15) Closest distance sectors [0,1]
    this->normalized_observations.closest_distance_sector.resize(this->observations.closest_distance_sector.size());
    for (size_t i = 0; i < this->observations.closest_distance_sector.size(); ++i)
    {
        this->normalized_observations.closest_distance_sector[i] =
            std::min(this->observations.closest_distance_sector[i] / (lidar_max_range/6.0f), 1.0f);
    }
}


void ObservationsServerNode::publishObservations()
{
    auto observations_msg = custom_interfaces::msg::Observations();
    // 1)  Distance Goal
    observations_msg.distance_goal = this->observations.distance_goal;
    // 2)  Distance to next path marker
    observations_msg.distance_next_marker = this->observations.distance_next_marker;
    // 3)  Relative Direction to next path marker
    observations_msg.relative_direction_next_marker = this->observations.relative_direction_next_marker;
    // 4) Linear velocity
    observations_msg.linear_velocity = this->observations.linear_velocity;
    // 5) Angular velocity
    observations_msg.angular_velocity = this->observations.angular_velocity;
    // 6-15) Closest distance sectors
    observations_msg.closest_distance_sector = this->observations.closest_distance_sector;
    // Publish the observations message
    this->observations_publisher_->publish(observations_msg);
}


void ObservationsServerNode::publishNormalizedObservations()
{
    auto normalized_observations_msg = custom_interfaces::msg::Observations();
    // 1)  Distance Goal
    normalized_observations_msg.distance_goal = this->normalized_observations.distance_goal;
    // 2)  Distance to next path marker
    normalized_observations_msg.distance_next_marker = this->normalized_observations.distance_next_marker;
    // 3)  Relative Direction to next path marker
    normalized_observations_msg.relative_direction_next_marker = this->normalized_observations.relative_direction_next_marker;
    // 4) Linear velocity
    normalized_observations_msg.linear_velocity = this->normalized_observations.linear_velocity;
    // 5) Angular velocity
    normalized_observations_msg.angular_velocity = this->normalized_observations.angular_velocity;
    // 6-15) Closest distance sectors
    normalized_observations_msg.closest_distance_sector = this->normalized_observations.closest_distance_sector;
    // Publish the normalized observations message
    this->normalized_observations_publisher_->publish(normalized_observations_msg);
}
