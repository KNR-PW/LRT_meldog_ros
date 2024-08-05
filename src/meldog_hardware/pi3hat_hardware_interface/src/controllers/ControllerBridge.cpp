#include "controllers/ControllerBridge.hpp"
//#include "../../include/controllers/ControllerBridge.hpp"

using namespace controller_interface;
using mjbots::pi3hat::CanFrame;


ControllerBridge::ControllerBridge(
     std::unique_ptr<ControllerWrapper> wrapper, 
     const ControllerParameters& params): 
     wrapper_(std::move(wrapper)), params_(params){}
     
ControllerBridge::ControllerBridge(ControllerBridge&& other_controller):
    wrapper_(std::move(other_controller.wrapper_)), params_(other_controller.params_) {}

ControllerBridge& ControllerBridge::operator=(ControllerBridge&& other_controller)
{
    if (this != &other_controller)
    {
        this->wrapper_ = std::move(other_controller.wrapper_);
        this->params_ = other_controller.params_;
    }
    return *this;
}

void ControllerBridge::make_command(CanFrame& tx_frame, ControllerCommand& command) const
{
    command.position_ = params_.direction_* std::clamp(command.position_,
     params_.position_min_, params_.position_max_) + params_.position_offset_;
    command.velocity_ = params_.direction_* std::clamp(command.velocity_, -params_.velocity_max_, params_.velocity_max_);
    command.torque_ = params_.direction_* std::clamp(command.torque_, -params_.torque_max_, params_.torque_max_);

    tx_frame.expect_reply = true;
    
    wrapper_->command_to_tx_frame(tx_frame, command);
    
}

void ControllerBridge::get_state(const CanFrame& rx_frame, ControllerState& state) const
{
    wrapper_->rx_frame_to_state(rx_frame, state);
    state.position_ = params_.direction_ * (state.position_ - params_.position_offset_);
    state.velocity_ = params_.direction_ * state.velocity_;
    state.torque_ = params_.direction_ * state.torque_; 
}

void ControllerBridge::initialize(CanFrame& tx_frame) const
{
    tx_frame.expect_reply = true;
    wrapper_->init_to_tx_frame(tx_frame);
}

void ControllerBridge::start_up(CanFrame& tx_frame, ControllerCommand& command) const
{
    command.position_ = params_.direction_* std::clamp(command.position_,
     params_.position_min_, params_.position_max_) + params_.position_offset_;
    command.velocity_ = params_.direction_* std::clamp(command.velocity_, -params_.velocity_max_, params_.velocity_max_);
    command.torque_ = params_.direction_* std::clamp(command.torque_, -params_.torque_max_, params_.torque_max_);

    tx_frame.expect_reply = true;
    wrapper_->start_pos_to_tx_frame(tx_frame, command);
}

ControllerParameters ControllerBridge::get_params()
{
    return params_;
}