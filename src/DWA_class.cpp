#include <galax_navigation/DWA_class.hpp>

DwaNode::DwaNode(const std::string & node_name) : Node(node_name)
{   
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
    // ros2 topic pub /dwa_parameters custom_interfaces/msg/Dwa "{alpha: 0.8, beta: 0.2, gamma: 0.0, delta: 0.0}"
    // ros2 topic pub /dwa_parameters custom_interfaces/msg/Dwa "{alpha: 0.0, beta: 0.0, gamma: 0.0, delta: 0.0}"

    this->observations_subscription_ = this->create_subscription<custom_interfaces::msg::Observations>(
        "/observations", 
        1,
        std::bind(&DwaNode::observationsCallback, this, std::placeholders::_1));

    this->normalized_observations_subscription_ = this->create_subscription<custom_interfaces::msg::Observations>(
        "/normalized_observations", 
        1,
        std::bind(&DwaNode::observationsCallback, this, std::placeholders::_1));

    this->lidar_subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", 
        1,
        std::bind(&DwaNode::lidarCallback, this, std::placeholders::_1));

    // Initialize timer for DWA calculations
    this->dwa_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100), // Adjust the frequency as needed
        std::bind(&DwaNode::dwaTimerCallback, this));

    // Initialize publishers
    this->cmd_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(
        "/differential_controller/cmd_vel_unstamped", 
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
    
}


void DwaNode::observationsCallback(const custom_interfaces::msg::Observations::SharedPtr msg)
{
    this->observations_ = msg;
}


void DwaNode::normalizedObservationsCallback(const custom_interfaces::msg::Observations::SharedPtr msg)
{
    this->normalized_observations_ = msg;
}


void DwaNode::lidarCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
    this->lidar_data_ = msg;
}


// DWA
void DwaNode::dwaTimerCallback()
{
    // Check if data from observations, normalized_observations and lidar is available
    if (!this->observations_ || !this->normalized_observations_ || !this->lidar_data_) {
        RCLCPP_WARN(this->get_logger(), "Waiting for observations, normalized observations, or lidar data.");
        return; // Not enough data to perform DWA
    }
    else
    {
        RCLCPP_INFO(this->get_logger(), "DWA ready to calculate command.");
    }
    // Calculate the DWA command
    geometry_msgs::msg::Twist cmd_vel = this->calculateDWA();

    // TODO
    // Implement a velocity smoother here
    // For now, just publish the command directly

    // Publish the command
    this->cmd_publisher_->publish(cmd_vel);
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
            float heading_score = calculateHeadingScore();
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
    pcl_cloud.header.frame_id = "pioneer3dx_base_link";
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
    pointcloud_msg.header.frame_id = "pioneer3dx_base_link";

    // Publish
    this->dwa_map_publisher_->publish(pointcloud_msg);


    geometry_msgs::msg::Twist cmd_vel;
    cmd_vel.linear.x = v_opt;
    cmd_vel.angular.z = omega_opt;
    return cmd_vel;
}


float DwaNode::calculateHeadingScore()
{
    // Use the normalized heading error from observations (already in [-1, 1])
    // The closer to 0, the better aligned we are.
    if (this->normalized_observations_) {
        float heading_error = this->normalized_observations_->relative_direction_next_marker; // [-1, 1]
        // Convert error to a score in [0, 1], where 1 is best alignment
        return 1.0f - std::fabs(heading_error);
    }
    return 0.0f;
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


