#ifndef _ACTUATOR_WRAPPER_BASE_
#define _ACTUATOR_WRAPPER_BASE_

#include "pi3hat/pi3hat.h"

template<class Derived>
class ActuatorWrapperBase
{
    protected:
    /* pi3hat CAN frames */
    mjbots::pi3hat::CanFrame& tx_frame_;
    mjbots::pi3hat::CanFrame& rx_frame_;
    public:

    /* Constructor: takes CanFrame for later editing*/
    ActuatorWrapperBase(mjbots::pi3hat::CanFrame& tx_frame, mjbots::pi3hat::CanFrame& rx_frame): 
    tx_frame_(tx_frame), rx_frame_(rx_frame) {};

    /* Used for CRTP interface */
    Derived& derived()
    {
        return static_cast<Derived&>(*this);
    };

    /* Static virtual method for preparing TX CAN frame */
    void make_position(double position, double velocity, double feedforward_torque)
    {
        derived().make_position(position, velocity, feedforward_torque);
    };
    
};



#endif