#ifndef _MOTOR_WRAPPER_BASE_
#define _MOTOR_WRAPPER_BASE_

#include "../pi3hat/pi3hat.h"
#include <cmath>

/*  
    Base Actuator Wrapper class, used for wrapping actuator API with simple CRTP interface 
    to create CAN frames for pi3hat Input structure. Note that it uses static polymorphism, 
    so remember to change instance of your derived class in pi3hat hardware interface files 
    (only for creation of an object).
*/
namespace actuator_wrappers
{

/* Structure for basic actuator command */
struct ActuatorCommand
{
    double position_;   /* [radians] */
    double velocity_;   /* [radians/s] */
    double torque_;     /* [Nm] */
};

/* Structure for basic actuator state */
struct ActuatorState
{
    double position_;   /* [radians] */
    double velocity_;   /* [radians/s] */
    double torque_;     /* [Nm] */
    int temperature_;   /* [Celcius] */
    bool fault = false;
};

struct ActuatorParameters
{
    int id;
    int bus;
    double position_max_;
    double position_min_;
    double velocity_max_;
    double torque_max_;
    int direction_;
};

template<class Derived>
class ActuatorWrapperBase
{
    private:

    ActuatorParameters params_;

    /* Used for CRTP interface */
    Derived& derived()
    {
        return static_cast<Derived&>(*this);
    };

    protected:
    
    /* pi3hat CAN frames */
    using CanFrame = mjbots::pi3hat::CanFrame;

    public:
    
    /* Constructor: takes CanFrame for later editing*/
    ActuatorWrapperBase(ActuatorParameters& params): params_(params) {};

    /* Static virtual method for starting actuators */
    void init(CanFrame& tx_frame)
    {
        derived().init(tx_frame);
    };

    /* Static virtual method for preparing TX CAN frame from ActuatorCommand */
    void command_to_tx_frame(CanFrame& tx_frame, ActuatorCommand& command)
    {
        tx_frame.id = params_.id;
        tx_frame.bus  = params_.bus;
        command.position_ = params_.direction_* (fmax(command.position_, params_.position_min_), params_.position_max_);
        command.velocity_ = params_.direction_* fmin(fmax(command.velocity_, -params_.velocity_max_), params_.velocity_max_);
        command.torque_ = params_.direction_* fmin(fmax(command.torque_, -params_.torque_max_), params_.torque_max_);

        derived().make_position(tx_frame, command);
    };

    /* Static virtual method for preparing ActuatorState form RX CAN frame */
    void rx_frame_to_state(CanFrame& rx_frame, ActuatorState& state)
    {
        if(rx_frame.id != params_.id) return; /* This should not happen! (map frame to wrapper first) */
        derived().make_position(rx_frame, state);
        state.position_ = params_.direction_ * state.position_;
        state.velocity_ = params_.direction_ * state.velocity_;
        state.torque_ = params_.direction_ * state.torque_; 
    };
    
};
};


#endif