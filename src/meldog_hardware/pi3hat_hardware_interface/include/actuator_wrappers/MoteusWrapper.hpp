#ifndef _MOTEUS_ACTUATOR_WRAPPER_
#define _MOTEUS_ACTUATOR_WRAPPER_


#include "moteus/moteus.h"
#include "ActuatorWrapperBase.hpp"

class MoteusWrapper: public ActuatorWrapperBase<MoteusWrapper>, protected mjbots::moteus::Controller
{
    private:
    /* Command structure for moteus object*/
    mjbots::moteus::PositionMode::Command position_command_;

    public:
    /* Create Moteus Wrapper from existing frames */
    MoteusWrapper(const mjbots::moteus::Controller::Options& options = {}, 
    mjbots::pi3hat::CanFrame& tx_frame,mjbots::pi3hat::CanFrame& rx_frame,
     mjbots::moteus::PositionMode::Command command);

    /* Static override */
    void make_position(double position, double velocity, double feedforward_torque);
};




#endif