#include "joint_controller_ros2_control/pid_controller/pid_controller.hpp"

using namespace pid_controller;

double PidController::calculateProportionalEffort(double _position_error, double _proportional_scale)
{
    return _position_error * PidController::pid_params_.proportional_coef_ * _proportional_scale;
}

double PidController::calculateDerivativeEffort(double _velocity_error, double _derivative_scale)
{
    return _velocity_error * PidController::pid_params_.derivative_coef_ * _derivative_scale;
}

double PidController::calculateIntegralEffort(double _position_error)
{
    PidController::position_integrator_ += PidController::pid_params_.integral_coef_ * _position_error*PidController::time_step_;
    if (PidController::position_integrator_ > PidController::pid_params_.integration_limit_)
    {
        return PidController::pid_params_.integration_limit_;
    }
    else if (PidController::position_integrator_ > PidController::pid_params_.integration_limit_)
    {
        return -PidController::pid_params_.integration_limit_;
    }
    else
    {
        return PidController::position_integrator_;
    }
}

double PidController::calculateEffort(double _position_error, double _velocity_error,
         double _feedforward_effort, double _proportional_scale, double _derivative_scale)
{
    double P = PidController::calculateProportionalEffort(_position_error, _proportional_scale);
    double D = PidController::calculateDerivativeEffort(_velocity_error, _derivative_scale);
    double I = PidController::calculateIntegralEffort(_position_error);
    return P + I + D + _feedforward_effort;
}

PidController::PidController(PidParameters _pid_params, double _frequency)
{
    PidController::pid_params_ = _pid_params;
    PidController::time_step_ = 1/_frequency;
}