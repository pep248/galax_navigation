#include <galax_navigation/DWA_class.hpp>

DwaNode::DwaNode(const std::string & node_name) : Node(node_name)
{   
    // Initialize parameter listener
    this->dwa_parameters_listener_ = std::make_shared<dwa_parameters_file::ParamListener>(this->shared_from_this());
    this->dwa_parameters_ = std::make_shared<dwa_parameters_file::Params::DwaParams>(this->shared_from_this());
    this->robot_parameters_ = std::make_shared<dwa_parameters_file::Params::RobotConstantParams>(this->shared_from_this());

    // Initialize DWA parameters
    this->dwa_parameters_instance_ = std::make_shared<DwaParametersClass>();
    this->robot_parameters_instance_ = std::make_shared<RobotParametersClass>();

    // Initialize subscribers
    DWA_subscription_ = this->create_subscription<custom_interfaces::msg::Dwa>(
        "dwa_parameters", 
        10,
        std::bind(&DwaNode::dwaCallback, this, std::placeholders::_1));

    // Initialize publishers
    cmd_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(
        "cmd_vel", 
        rclcpp::QoS(10));

    // Initialize timer -> TODO -> create the timer once the parametsrs have been loaded
    initial_check_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&DwaNode::initialCheckTimerCallback, this));
}

void DwaNode::get_params()
{
    // DWA parameters
    this->dwa_parameters_instance_->alpha        = dwa_parameters_->alpha;
    this->dwa_parameters_instance_->beta         = dwa_parameters_->beta;
    this->dwa_parameters_instance_->delta        = dwa_parameters_->delta;
    this->dwa_parameters_instance_->gamma        = dwa_parameters_->gamma;
    this->dwa_parameters_instance_->time_horizon = dwa_parameters_->time_horizon;
    this->dwa_parameters_instance_->n_samples    = dwa_parameters_->n_samples;

    // Robot constant parameters
    this->robot_parameters_instance_->mass      = robot_parameters_->mass;
    this->robot_parameters_instance_->inertia   = robot_parameters_->inertia;
    this->robot_parameters_instance_->max_speed = robot_parameters_->max_speed;
    this->robot_parameters_instance_->max_accel = robot_parameters_->max_accel;
    this->robot_parameters_instance_->max_omega = robot_parameters_->max_omega;
    this->robot_parameters_instance_->max_alpha = robot_parameters_->max_alpha;

    params_loaded = true;
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
    int N = dwa.n_samples;
    float deltaV = (v_max - v_min) / std::max(1, N-1);
    float deltaW = (omega_max - omega_min) / std::max(1, N-1);

    float best_score = -1e9;
    float v_opt = 0.0, omega_opt = 0.0;

    for (int i = 0; i <= N; ++i)
    {
        float v = v_min + i * deltaV;
        for (int j = 0; j <= N; ++j)
        {
            float omega = omega_min + j * deltaW;
            RCLCPP_INFO(this->get_logger(), "Evaluating v: %.2f, omega: %.2f", v, omega);
            // Simulate motion
            float theta_sim, x_sim, y_sim;
            float theta = robot.robot_pose.theta;
            float x = robot.robot_pose.x;
            float y = robot.robot_pose.y;

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

            // --- Score calculation ---
            float heading_score = calculateHeadingScore(x_sim, y_sim, theta_sim);
            float obstacle_score = calculateObstacleScore(x_sim, y_sim);
            float velocity_score = (robot.max_speed > 0) ? v / robot.max_speed : 0.0;
            float energy_score = 1.0 - std::min((robot.mass * v * v + robot.inertia * omega * omega) / (2.0 * robot.mass * robot.max_speed * robot.max_speed), 1.0);

            float score = dwa.alpha * heading_score +
                           dwa.beta * obstacle_score +
                           dwa.gamma * velocity_score +
                           dwa.delta * energy_score;

            if (score > best_score)
            {
                best_score = score;
                v_opt = v;
                omega_opt = omega;
            }
        }
    }

    geometry_msgs::msg::Twist cmd_vel;
    cmd_vel.linear.x = v_opt;
    cmd_vel.angular.z = omega_opt;
    return cmd_vel;
}

// Example stubs for scoring functions (implement according to your environment)
float DwaNode::calculateHeadingScore(float x_sim, float y_sim, float theta_sim)
{
    // TODO: Implement using your path/goal
    // Example: return value between 0 and 1
    return 1.0;
}

float DwaNode::calculateObstacleScore(float x_sim, float y_sim)
{
    // TODO: Implement using your lidar/occupancy grid
    // Example: return value between 0 and 1
    return 1.0;
}

void DwaNode::dwaCallback(const custom_interfaces::msg::Dwa::SharedPtr msg)
{
    // Callback implementation
}

void DwaNode::dwaTimerCallback()
{
    // Timer callback implementation
}
