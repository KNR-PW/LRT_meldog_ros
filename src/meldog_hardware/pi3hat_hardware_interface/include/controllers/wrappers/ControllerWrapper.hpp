#ifndef _CONTROLLER_BRIDGE_H_
#define _CONTROLLER_BRIDGE_H_

#include "../ControllerBridge.hpp"


namespace controller_interface
{

class ControllerWrapper
{
    protected:
    ControllerParameters params_;
    
    public:
    using CanFrame = mjbots::pi3hat::CanFrame;

    ControllerWrapper(const ControllerParameters& params);
    virtual void command_to_tx_frame(CanFrame& tx_frame, const ControllerCommand& command) = 0;
    virtual void rx_frame_to_state(const CanFrame& rx_frame, ControllerState& state) = 0;
    virtual void stop_to_tx_frame(CanFrame& tx_frame) = 0;
    virtual ~ControllerWrapper() = 0;

};

};

#endif