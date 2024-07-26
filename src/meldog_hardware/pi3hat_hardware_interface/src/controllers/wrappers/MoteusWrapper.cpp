#include "../../../include/controllers/wrappers/MoteusWrapper.hpp"

using namespace controller_interface;

MoteusWrapper::MoteusWrapper(
        const mjbots::moteus::Controller::Options& options,
        const mjbots::moteus::PositionMode::Command& command):

        ControllerWrapper(), 
        position_command_(command),
        moteus_controller_(mjbots::moteus::Controller(options)) {} 


void MoteusWrapper::command_to_tx_frame(CanFrame& tx_frame, const ControllerCommand& command) 
{
    /* Change command values */
    position_command_.position = command.position_ * radians_to_rotation;
    position_command_.velocity = command.velocity_ * radians_to_rotation;
    position_command_.feedforward_torque = command.torque_;

    /* Create CANFD frame*/
    mjbots::moteus::CanFdFrame can_fd_frame = moteus_controller_.MakePosition(position_command_);
    
    /* Copy data from CANFD frame to CAN frame */
    tx_frame.id = can_fd_frame.arbitration_id;
    tx_frame.bus = can_fd_frame.bus;
    tx_frame.size = can_fd_frame.size;
    std::memcpy(tx_frame.data, can_fd_frame.data, can_fd_frame.size);
}

void MoteusWrapper::rx_frame_to_state(const CanFrame& rx_frame, ControllerState& state) 
{
    /* Parse data from RX CAN frame to Result object */
    if(((rx_frame.id >> 8) & 0x7f) != moteus_controller_.options().id) return; /* This should not happen! (map frame to wrapper first) */

    mjbots::moteus::Query::Result result = mjbots::moteus::Query::Parse(rx_frame.data, rx_frame.size);
    state.position_ = result.position * rotation_to_radians;
    state.velocity_ = result.velocity * rotation_to_radians;
    state.torque_ = result.torque;
    state.temperature_ = result.temperature;
    state.fault = result.fault;
}

void MoteusWrapper::stop_to_tx_frame(CanFrame& tx_frame) 
{
    /* create CANFD frame*/
    mjbots::moteus::CanFdFrame can_fd_frame = moteus_controller_.MakeStop();

    /* Copy data from CANFD frame to CAN frame*/
    tx_frame.id = can_fd_frame.arbitration_id;
    tx_frame.bus = can_fd_frame.bus;
    tx_frame.size = can_fd_frame.size;
    std::memcpy(tx_frame.data, can_fd_frame.data, can_fd_frame.size);
}
