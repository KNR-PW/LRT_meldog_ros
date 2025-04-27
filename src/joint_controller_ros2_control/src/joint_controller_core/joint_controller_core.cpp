#include "joint_controller_ros2_control/joint_controller_core/joint_controller_core.hpp"

using namespace joint_controller_core;

JointControllerCore::JointControllerCore(JointParameters _joint_params,
 pid_controller::PidParameters _pid_params, double _frequency): 
    joint_params_(_joint_params), pid_controller_(_pid_params, _frequency){}

double JointControllerCore::calculateEffort(const JointCommands& _joint_command,const JointStates& _joint_state)
{
    double desired_position = std::clamp(_joint_command.desired_position_ + joint_params_.position_offset_,
         joint_params_.position_min_, joint_params_.position_max_);
    double desired_velocity = std::clamp(_joint_command.desired_velocity_,
     -joint_params_.velocity_max_, joint_params_.velocity_max_);

    double position_error = desired_position - _joint_state.position_;
    double velocity_error = desired_velocity - _joint_state.velocity_;
    double effort =  pid_controller_.calculateEffort(position_error, velocity_error,
     _joint_command.feedforward_effort_, _joint_command.kp_scale_, _joint_command.kd_scale_);

    return std::clamp(effort, -joint_params_.effort_max_, joint_params_.effort_max_);
}