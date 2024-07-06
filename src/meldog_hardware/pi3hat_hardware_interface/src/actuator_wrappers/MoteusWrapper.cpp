#include "../include/actuator_wrappers/MoteusWrapper.hpp"


/*
    Moteus Actuator Wrapper. It uses API from moteus repository to communicate with pi3hat interface.
    https://github.com/mjbots/moteus

*/
MoteusWrapper::MoteusWrapper(const mjbots::moteus::Controller::Options& options = {}, 
mjbots::pi3hat::CanFrame& tx_frame, mjbots::pi3hat::CanFrame& rx_frame, 
mjbots::moteus::PositionMode::Command command): 
ActuatorWrapperBase<MoteusWrapper>(tx_frame, rx_frame), mjbots::moteus::Controller(options), position_command_(command)
{
    /* Prepare CAN tx frame*/
    ActuatorWrapperBase<MoteusWrapper>::tx_frame_.id = options.id; 
    ActuatorWrapperBase<MoteusWrapper>::tx_frame_.bus = options.bus;       // Copies values from options structure

    ActuatorWrapperBase<MoteusWrapper>::tx_frame_.expect_reply = true;     // Expect reply from the same bus
}


void MoteusWrapper::make_position(double position, double velocity, double feedforward_torque)
{
    /* Change command values */
    position_command_.position = position;
    position_command_.velocity = velocity;
    position_command_.feedforward_torque = feedforward_torque;

    /* create CANFD frame*/
    mjbots::moteus::CanFdFrame can_fd_frame = mjbots::moteus::Controller::MakePosition(position_command_);
    
    /* Copy data from CANFD frame to CAN frame*/
    tx_frame_.size = can_fd_frame.size;
    std::memcpy(tx_frame_.data, can_fd_frame.data, can_fd_frame.size);
}