#ifndef _MOTEUS_ACTUATOR_WRAPPER_
#define _MOTEUS_ACTUATOR_WRAPPER_


#include "../moteus/moteus.h"
#include "ActuatorWrapperBase.hpp"

namespace actuator_wrappers
{
class MoteusWrapper: public ActuatorWrapperBase<MoteusWrapper>, protected mjbots::moteus::Controller
{
    private:

    /* Const coefficients for easy radians - rotations transform */
    static const double rotation_to_radians = 2 * M_PI;
    static const double radians_to_rotation = 1 / (2 * M_PI); /* Multiplying is faster than dividing */

    /* Command structure for moteus object*/
    mjbots::moteus::PositionMode::Command position_command_;

    public:
    /* Create Moteus Wrapper from existing frames */
    MoteusWrapper(const ActuatorParameters& params, 
    mjbots::moteus::Controller::Options& options,
    mjbots::moteus::PositionMode::Command& command);

    /* Static override */
    void init(CanFrame& tx_frame);
    void command_to_tx_frame(CanFrame& tx_frame, ActuatorCommand& command);
    void rx_frame_to_state(const CanFrame& rx_frame, ActuatorState& state);
};
};




#endif