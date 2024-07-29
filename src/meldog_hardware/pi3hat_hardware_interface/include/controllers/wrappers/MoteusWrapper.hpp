#ifndef _MOTEUS_WRAPPER_HPP_
#define _MOTEUS_WRAPPER_HPP_

#include "ControllerWrapper.hpp"
#include "../../3rd_libs/moteus/moteus.h"

namespace controller_interface
{

class MoteusWrapper: public ControllerWrapper
{
    private:

    /* Const coefficients for easy radians - rotations transform */
    constexpr static double rotation_to_radians_ = 2 * M_PI;
    constexpr static double radians_to_rotation_ = 1 / (2 * M_PI); /* Multiplying is faster than dividing */
    constexpr static double startup_coefficient_ = 0.05; /* For slow start-up */

    /* Command structure for moteus object*/
    mjbots::moteus::PositionMode::Command position_command_;
    mjbots::moteus::Controller moteus_controller_;
    
    public:
    using CanFrame = mjbots::pi3hat::CanFrame;

    MoteusWrapper( 
        const mjbots::moteus::Controller::Options& options,
        const mjbots::moteus::PositionMode::Command& command);
    void command_to_tx_frame(CanFrame& tx_frame, const ControllerCommand& command) override;
    void rx_frame_to_state(const CanFrame& rx_frame, ControllerState& state) override;
    void init_to_tx_frame(CanFrame& tx_frame) override;
    void start_pos_to_tx_frame(CanFrame& tx_frame, const ControllerCommand& command) override;

    // ~MoteusWrapper() override = default;

};

};

#endif