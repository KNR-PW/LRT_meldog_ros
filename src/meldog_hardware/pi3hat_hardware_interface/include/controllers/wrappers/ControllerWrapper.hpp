#ifndef _CONTROLLER_WRAPPER_HPP_
#define _CONTROLLER_WRAPPER_HPP_

#include "../ControllerStructures.hpp"
#include "3rd_libs/pi3hat/pi3hat.h"

namespace controller_interface
{

class ControllerWrapper
{
    protected:
    
    public:
    using CanFrame = mjbots::pi3hat::CanFrame;

    ControllerWrapper() = default;
    ControllerWrapper(const ControllerWrapper& other) = default;
    ControllerWrapper(ControllerWrapper&& other) = default;
    ControllerWrapper& operator=(const ControllerWrapper& other) = default;
    ControllerWrapper& operator=(ControllerWrapper&& other) = default;
    
    virtual void command_to_tx_frame(CanFrame& tx_frame, const ControllerCommand& command) = 0;
    virtual void query_to_tx_frame(CanFrame& tx_frame) = 0;
    virtual void rx_frame_to_state(const CanFrame& rx_frame, ControllerState& state) = 0;
    virtual void init_to_tx_frame(CanFrame& tx_frame) = 0;
    virtual int get_id_from_rx_frame(const CanFrame& rx_frame) = 0;

    virtual ~ControllerWrapper() = default;

};

};

#endif