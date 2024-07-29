#ifndef _CONTROLLER_WRAPPER_HPP_
#define _CONTROLLER_WRAPPER_HPP_

#include "../ControllerStructures.hpp"
#include "../../3rd_libs/pi3hat/pi3hat.h"

namespace controller_interface
{

class ControllerWrapper
{
    protected:
    
    public:
    using CanFrame = mjbots::pi3hat::CanFrame;

    ControllerWrapper() = default;
    virtual void command_to_tx_frame(CanFrame& tx_frame, const ControllerCommand& command) = 0;
    virtual void rx_frame_to_state(const CanFrame& rx_frame, ControllerState& state) = 0;
    virtual void init_to_tx_frame(CanFrame& tx_frame) = 0;
    virtual void start_pos_to_tx_frame(CanFrame& tx_frame, const ControllerCommand& command) = 0;

    virtual ~ControllerWrapper() = default;

};

};

#endif