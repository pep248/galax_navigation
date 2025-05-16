#include <galax_navigation/DWA_class.hpp>

DWA_Node::DWA_Node()
{   
    // Initialize parameter listener
    this->dwa_parameters_listener_ = std::make_shared<dwa_parameters::ParamListener>(this->shared_from_this());
    this->dwa_parameters_ = std::make_shared<dwa_parameters::DWA_parameters>(this->shared_from_this());

    // Initialize DWA parameters
    this->dwa_dynamic_parameters_ = std::make_shared<DWA_parameters_class>();
}

void DWA_Node::get_params()
{
    dwa_parameters_instance->max_speed = dwa_parameters_->max_speed;
    dwa_parameters_instance->min_speed = dwa_parameters_->min_speed;
    dwa_parameters_instance->max_accel = dwa_parameters_->max_accel;
    dwa_parameters_instance->max_decel = dwa_parameters_->max_decel;
    dwa_parameters_instance->max_yaw_rate = dwa_parameters_->max_yaw_rate;
    dwa_parameters_instance->min_yaw_rate = dwa_parameters_->min_yaw_rate;
    dwa_parameters_instance->max_yaw_accel = dwa_parameters_->max_yaw_accel;
    dwa_parameters_instance->max_yaw_decel = dwa_parameters_->max_yaw_decel;
    dwa_parameters_instance->min_turning_radius = dwa_parameters_->min_turning_radius;
    dwa_parameters_instance->heading_coeff = dwa_parameters_->heading_coeff;
    dwa_parameters_instance->obstacle_coeff = dwa_parameters_->obstacle_coeff;
    dwa_parameters_instance->velocity_coeff = dwa_parameters_->velocity_coeff;
    dwa_parameters_instance->energy_coeff = dwa_parameters_->energy_coeff;
    dwa_parameters_instance->time_horizon = dwa_parameters_->time_horizon;
    dwa_parameters_instance->velocity_samples = dwa_parameters_->velocity_samples;
}

void DWA_Node::dwa_callback(const custom_interfaces::msg::DWA::SharedPtr msg)
{
    // Callback implementation
}

void DWA_Node::timer_callback()
{
    // Timer callback implementation
}

DWA_parameters_class::DWA_parameters_class()
{
    // Constructor implementation
}
