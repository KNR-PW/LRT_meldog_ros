#ifndef _MOTEUS_ACTUATOR_WRAPPER_
#define _MOTEUS_ACTUATOR_WRAPPER_


#include "moteus/moteus.h"
#include "ActuatorWrapperBase.hpp"

namespace actuator_wrappers
{
class MoteusWrapper: public ActuatorWrapperBase<MoteusWrapper>, protected mjbots::moteus::Controller
{
    private:
    /* Command structure for moteus object*/
    mjbots::moteus::PositionMode::Command position_command_;
    mjbots::moteus::PositionMode::Format potision_format_;

    public:
    /* Create Moteus Wrapper from existing frames */
    MoteusWrapper(ActuatorParameters params, 
    mjbots::moteus::Controller::Options& options,
    mjbots::moteus::PositionMode::Format format,
    mjbots::moteus::PositionMode::Command command);

    /* Static override */
    void command_to_tx_frame(CanFrame& tx_frame, ActuatorCommand& command);
    void rx_frame_to_state(CanFrame& rx_frame, ActuatorState& state);
};
};




#endif