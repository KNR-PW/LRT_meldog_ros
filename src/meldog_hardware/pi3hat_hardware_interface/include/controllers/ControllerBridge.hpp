#ifndef _CONTROLLER_BRIDGE_HPP_
#define _CONTROLLER_BRIDGE_HPP_

#include "../3rd_libs/pi3hat/pi3hat.h"
#include "wrappers/ControllerWrapper.hpp"
#include "ControllerStructures.hpp"
#include <memory>
#include <algorithm>

namespace controller_interface
{

class ControllerBridge
{
    private:

    using CanFrame = mjbots::pi3hat::CanFrame;

    const std::unique_ptr<ControllerWrapper> wrapper_;
    ControllerParameters params_;


    public:
    ControllerBridge(std::unique_ptr<ControllerWrapper> wrapper, 
     const ControllerParameters& params);

    void make_command(CanFrame& tx_frame, ControllerCommand& command) const; //POMYSL JESZCZE O TYM
    void get_state(const CanFrame& rx_frame, ControllerState& state) const;
    void initialize(CanFrame& tx_frame) const;
    void start_up(CanFrame& tx_frame, ControllerCommand& command) const;
};

};
#endif