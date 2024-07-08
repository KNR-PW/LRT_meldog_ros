#ifndef _MOTOR_WRAPPER_BASE_
#define _MOTOR_WRAPPER_BASE_

#include "pi3hat/pi3hat.h"


/*  
    Base Motor Wrapper class, used for wrapping actuator API with simple CRTP interface 
    to create CAN frames for pi3hat Input structure. Note that it uses static polymorphism, 
    so remember to change instance of your derived class in pi3hat hardware interface files 
    (only for creation of an object).
*/
template<class Derived>
class MotorWrapperBase
{
    private:

    /* Used for CRTP interface */
    Derived& derived()
    {
        return static_cast<Derived&>(*this);
    };

    protected:
    
    /* pi3hat CAN frames */
    mjbots::pi3hat::CanFrame& tx_frame_;
    mjbots::pi3hat::CanFrame& rx_frame_;

    public:
    
    /* Constructor: takes CanFrame for later editing*/
    MotorWrapperBase(mjbots::pi3hat::CanFrame& tx_frame, mjbots::pi3hat::CanFrame& rx_frame): 
    tx_frame_(tx_frame), rx_frame_(rx_frame) {};

    /* Static virtual method for preparing TX CAN frame */
    void make_position(double position, double velocity, double feedforward_torque)
    {
        derived().make_position(position, velocity, feedforward_torque);
    };
    
};



#endif