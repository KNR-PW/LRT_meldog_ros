#ifndef _MOTEUS_BRIDGE_H_
#define _MOTEUS_BRIDGE_H_

#include "ControllerBridge.hpp"
#include "../../moteus/moteus.h"

namespace controller_interface
{

class MoteusBridge: public ControllerBridge
{
    private:

    /* Const coefficients for easy radians - rotations transform */
    static const double rotation_to_radians = 2 * M_PI;
    static const double radians_to_rotation = 1 / (2 * M_PI); /* Multiplying is faster than dividing */

    /* Command structure for moteus object*/
    mjbots::moteus::PositionMode::Command position_command_;
    mjbots::moteus::Controller moteus_controller_;
    
    public:
    using CanFrame = mjbots::pi3hat::CanFrame;

    MoteusBridge(
        const ControllerParameters& params, 
        const mjbots::moteus::Controller::Options& options,
        const mjbots::moteus::PositionMode::Command& command);
    void command_to_tx_frame(CanFrame& tx_frame, const ControllerCommand& command) override;
    void rx_frame_to_state(const CanFrame& rx_frame, ControllerState& state) override;
    void stop_to_tx_frame(CanFrame& tx_frame) override;
    ~MoteusBridge() override = default;

};

};

#endif