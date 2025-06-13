#include <galax_navigation/DWA_class.hpp>

DwaNode::DwaNode(const std::string & node_name) : Node(node_name)
{   

    RCLCPP_INFO(get_logger(), "DwaNode initialized");

    // Parameters
    this->file_parameters_listener_ = std::make_shared<dwa_parameters_file::ParamListener>(this->get_node_parameters_interface());
    this->file_parameters_ = std::make_shared<dwa_parameters_file::Params>(this->file_parameters_listener_->get_params());
    this->dwa_parameters_instance_ = std::make_shared<DwaParametersClass>();
    this->robot_parameters_instance_ = std::make_shared<RobotParametersClass>();

    this->get_params();


    // Initialize subscribers
    this->dwa_subscription_ = this->create_subscription<custom_interfaces::msg::Dwa>(
        "/dwa_parameters", 
        1,
        std::bind(&DwaNode::updateDwaCallback, this, std::placeholders::_1));
    // ros2 topic pub /dwa_parameters custom_interfaces/msg/Dwa "{alpha: 0.8, beta: 0.2, gamma: 0.1, delta: 0.0}"
    // ros2 topic pub /dwa_parameters custom_interfaces/msg/Dwa "{alpha: 0.0, beta: 0.0, gamma: 0.0, delta: 0.0}"

    this->observations_subscription_ = this->create_subscription<custom_interfaces::msg::Observations>(
        "/observations", 
        1,
        std::bind(&DwaNode::observationsCallback, this, std::placeholders::_1));

    this->normalized_observations_subscription_ = this->create_subscription<custom_interfaces::msg::Observations>(
        "/normalized_observations", 
        1,
        std::bind(&DwaNode::normalizedObservationsCallback, this, std::placeholders::_1));

    this->lidar_subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", 
        1,
        std::bind(&DwaNode::lidarCallback, this, std::placeholders::_1));
    this->goal_reached_subscription_ = this->create_subscription<std_msgs::msg::Bool>(
        "/goal_reached", 
        1,
        std::bind(&DwaNode::goalReachedCallback, this, std::placeholders::_1));


    // Initialize timer for DWA calculations
    this->dwa_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100), // Adjust the frequency as needed
        std::bind(&DwaNode::dwaTimerCallback, this));

    // Initialize publishers
    this->cmd_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(
        "/cmd_vel_dwa", 
        rclcpp::QoS(1));
    this->dwa_map_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "/DWA/pointcloud2",
        rclcpp::QoS(rclcpp::KeepLast(0)).durability_volatile()
    );
}


void DwaNode::start()
{

}


void DwaNode::get_params()
{
    // DWA parameters
    this->dwa_parameters_instance_->alpha        = this->file_parameters_->DWA_params.alpha;
    this->dwa_parameters_instance_->beta         = this->file_parameters_->DWA_params.beta;
    this->dwa_parameters_instance_->gamma        = this->file_parameters_->DWA_params.gamma;
    this->dwa_parameters_instance_->delta        = this->file_parameters_->DWA_params.delta;
    this->dwa_parameters_instance_->time_horizon = this->file_parameters_->DWA_params.time_horizon;
    this->dwa_parameters_instance_->n_samples    = this->file_parameters_->DWA_params.n_samples;

    // Robot constant parameters
    this->robot_parameters_instance_->mass      = this->file_parameters_->robot_constant_params.mass;
    this->robot_parameters_instance_->inertia   = this->file_parameters_->robot_constant_params.inertia;
    this->robot_parameters_instance_->max_speed = this->file_parameters_->robot_constant_params.max_speed;
    this->robot_parameters_instance_->max_accel = this->file_parameters_->robot_constant_params.max_accel;
    this->robot_parameters_instance_->max_omega = this->file_parameters_->robot_constant_params.max_omega;
    this->robot_parameters_instance_->max_alpha = this->file_parameters_->robot_constant_params.max_alpha;

    RCLCPP_INFO(get_logger(), "DWA parameters loaded:");
    
}


// Callbacks
void DwaNode::updateDwaCallback(const custom_interfaces::msg::Dwa::SharedPtr msg)
{
    this->dwa_parameters_instance_->alpha        = msg->alpha;
    this->dwa_parameters_instance_->beta         = msg->beta;
    this->dwa_parameters_instance_->gamma        = msg->gamma;
    this->dwa_parameters_instance_->delta        = msg->delta;
    // RCLCPP_INFO(get_logger(), "DWA parameters updated: alpha=%.2f, beta=%.2f, gamma=%.2f, delta=%.2f",
    //             msg->alpha, msg->beta, msg->gamma, msg->delta);
    
}


void DwaNode::observationsCallback(const custom_interfaces::msg::Observations::SharedPtr msg)
{ 
    this->observations_ = msg;
    this->observations_ready_ = true;
}


void DwaNode::normalizedObservationsCallback(const custom_interfaces::msg::Observations::SharedPtr msg)
{
    this->normalized_observations_ = msg;
    this->normalized_observations_ready_ = true;
}


void DwaNode::lidarCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
    this->lidar_data_ = msg;
    this->lidar_data_ready_ = true;
}


void DwaNode::goalReachedCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
    if (msg->data)
    {
        RCLCPP_INFO(this->get_logger(), "Goal reached, stopping DWA calculations.");
        this->goal_reached = true;
        this->cmd_publisher_->publish(geometry_msgs::msg::Twist()); // Stop the robot
    }
    else
    {
        RCLCPP_WARN(this->get_logger(), "Goal not reached yet.");
        this->goal_reached = false;
    }
}

// DWA
void DwaNode::dwaTimerCallback()
{

    if (this->all_data_received == true)
    {
        if (this->goal_reached)
        {
            this->cmd_publisher_->publish(geometry_msgs::msg::Twist()); // Stop the robot
            return;
        }
        else
        {
            // Calculate the DWA command
            geometry_msgs::msg::Twist cmd_vel = this->calculateDWA();

            // TODO
            // Implement a velocity smoother here
            // For now, just publish the command directly

            // Publish the command
            this->cmd_publisher_->publish(cmd_vel);
        }

    }
    else
    {
        if (this->observations_ready_ && this->normalized_observations_ready_ && this->lidar_data_ready_)
        {
            RCLCPP_INFO(this->get_logger(), "All data received, starting DWA calculations.");
            this->all_data_received = true;
        }
        else
        {
            if (!this->observations_ready_)
            {
                RCLCPP_WARN(this->get_logger(), "Observations data not received yet.");
            }
            if (!this->normalized_observations_ready_)
            {
                RCLCPP_WARN(this->get_logger(), "Normalized observations data not received yet.");
            }
            if (!this->lidar_data_ready_)
            {
                RCLCPP_WARN(this->get_logger(), "Lidar data not received yet.");
            }
            
        }
    }

}


geometry_msgs::msg::Twist DwaNode::calculateDWA()
{
    
    // Get robot and DWA parameters
    auto& robot = *this->robot_parameters_instance_; // Reference to robot parameters
    auto& dwa = *this->dwa_parameters_instance_; // Reference to DWA parameters

    // Limits
    float v_max = std::min(robot.max_speed, robot.max_speed + robot.max_accel * dwa.time_horizon);
    float v_min = std::max(0.0f, -robot.max_accel * dwa.time_horizon);
    float omega_max = std::min(robot.max_omega, robot.max_omega + robot.max_alpha * dwa.time_horizon);
    float omega_min = std::max(-robot.max_omega, -robot.max_omega - robot.max_alpha * dwa.time_horizon);

    // Sampling
    int N = dwa.n_samples-1;
    float deltaV = (v_max - v_min) / std::max(1, N);
    float deltaW = (omega_max - omega_min) / std::max(1, N);

    float best_score = -1e9;
    float v_opt = 0.0, omega_opt = 0.0;
    // Store points for visualization
    std::vector<std::tuple<float, float, float, float>> dwa_points; // x, y, z, intensity

    for (int i = 0; i <= N; ++i)
    {
        float v = v_min + i * deltaV;
        for (int j = 0; j <= N; ++j)
        {
            float omega = omega_min + j * deltaW;
            // RCLCPP_INFO(this->get_logger(), "Evaluating v: %.2f, omega: %.2f", v, omega);
            // Simulate motion
            float theta_sim, x_sim, y_sim;
            float theta = 0;
            float x = 0;
            float y = 0;

            if (std::abs(omega) < 1e-6)
            {
                theta_sim = theta;
                x_sim = x + v * std::cos(theta) * dwa.time_horizon;
                y_sim = y + v * std::sin(theta) * dwa.time_horizon;
            }
            else
            {
                float radius = v / omega;
                theta_sim = theta + omega * dwa.time_horizon;
                float cx = x - radius * std::sin(theta);
                float cy = y + radius * std::cos(theta);
                x_sim = cx + radius * std::sin(theta_sim);
                y_sim = cy - radius * std::cos(theta_sim);
            }

            // Heading score
            float heading_score = calculateHeadingScore(x_sim, y_sim, theta_sim);
            // Obstacle score
            float obstacle_score = calculateObstacleScore(x_sim, y_sim);
            // Velocity score
            float velocity_score = 0.0f;
            if (robot.max_speed > 0.0f)
            {
                velocity_score = v / robot.max_speed; // Value between 0 (stopped) and 1 (max speed)
            }
            // Energy score
            float energy_score = 1.0 - std::min((robot.mass * v * v + robot.inertia * omega * omega) / (2.0 * robot.mass * robot.max_speed * robot.max_speed), 1.0);

            float score = dwa.alpha * heading_score +
                           dwa.beta * obstacle_score +
                           dwa.gamma * velocity_score +
                           dwa.delta * energy_score;

            dwa_points.emplace_back(x_sim, y_sim, 0.3f, score);

            // RCLCPP_INFO(get_logger(), "Scores - Heading: %.2f, Obstacle: %.2f, Velocity: %.2f, Energy: %.2f, Total: %.2f",
            //             heading_score, obstacle_score, velocity_score, energy_score, score);

            if (score > best_score)
            {
                best_score = score;
                v_opt = v;
                omega_opt = omega;
            }
        }
    }

    // Create a PCL point cloud
    pcl::PointCloud<pcl::PointXYZI> pcl_cloud;
    pcl_cloud.header.frame_id = "galax_base_link";
    pcl_cloud.is_dense = true;

    for (const auto& [x, y, z, score] : dwa_points)
    {
        pcl::PointXYZI pt;
        pt.x = x;
        pt.y = y;
        pt.z = z;
        pt.intensity = score;
        pcl_cloud.points.push_back(pt);
    }
    pcl_cloud.width = pcl_cloud.points.size();
    pcl_cloud.height = 1;

    // Convert to ROS PointCloud2
    sensor_msgs::msg::PointCloud2 pointcloud_msg;
    pcl::toROSMsg(pcl_cloud, pointcloud_msg);
    pointcloud_msg.header.stamp = this->now();
    pointcloud_msg.header.frame_id = "galax_base_link";

    // Publish
    this->dwa_map_publisher_->publish(pointcloud_msg);


    geometry_msgs::msg::Twist cmd_vel;
    cmd_vel.linear.x = v_opt;
    cmd_vel.angular.z = omega_opt;

    return cmd_vel;
}


float DwaNode::calculateHeadingScore(float x_sim, float y_sim, float theta_sim)
{
    // Check if observations_ is valid and has a heading value
    if (!std::isnan(this->observations_->relative_direction_next_marker) && !std::isnan(this->observations_->distance_next_marker))
    {
        // Calculate the x, y position of the next marker
        float next_marker_x = this->observations_->distance_next_marker * std::cos(this->observations_->relative_direction_next_marker);
        float next_marker_y = this->observations_->distance_next_marker * std::sin(this->observations_->relative_direction_next_marker);
        // Calculate the heading error
        float goal_heading = atan2(next_marker_y - y_sim, next_marker_x - x_sim);
        float heading_diff = abs(angdiff(theta_sim, goal_heading));
        // Convert error to a score in [0, 1], where 1 is best alignment
        return (this->PI - std::abs(heading_diff)) / this->PI;
    }


    RCLCPP_ERROR(get_logger(), "No valid heading information available in observations or normalized observations.");
    return 0.0f; // Default score if no valid heading information is available
}


float DwaNode::calculateObstacleScore(float x_sim, float y_sim)
{
    // Use the latest lidar data to compute the minimum distance from (x_sim, y_sim) to any lidar point
    if (!lidar_data_) {
        return 1.0f; // No data, safest score
    }

    const auto& scan = lidar_data_;
    float min_distance = std::numeric_limits<float>::max();

    // Robot pose (assume at (0,0,0) in its own frame for simulation)
    // If you want to use the real robot pose, adjust accordingly.

    for (size_t i = 0; i < scan->ranges.size(); ++i) {
        float range = scan->ranges[i];
        if (range < scan->range_min || range > scan->range_max) continue;
        float angle = scan->angle_min + i * scan->angle_increment;
        // Lidar point in robot frame
        float lx = range * std::cos(angle);
        float ly = range * std::sin(angle);
        // Distance from simulated point to this lidar point
        float dist = std::hypot(lx - x_sim, ly - y_sim);
        if (dist < min_distance)
        {
            min_distance = dist;
        }
    }

    // Cap and normalize as in your MATLAB code
    min_distance = std::max(min_distance, 0.4f);
    float score = std::min((min_distance - 0.4f) / (2.0f - 0.4f), 1.0f);
    return score;
}


float DwaNode::angdiff(float a, float b) {
    float d = a - b;
    while (d > this->PI) d -= 2.0f * this->PI;
    while (d < -this->PI) d += 2.0f * this->PI;
    return d;
}



