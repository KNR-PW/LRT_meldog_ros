#ifndef _CONTROLLER_STRUCTURES_HPP_
#define _CONTROLLER_STRUCTURES_HPP_

namespace controller_interface
{

/* Structure for basic actuator command */
struct ControllerCommand
{
    double position_ = 0;   /* [radians] */
    double velocity_  = 0;   /* [radians/s] */
    double torque_ = 0;     /* [Nm] */
};

/* Structure for basic actuator state */
struct ControllerState
{
    double position_ = 0;   /* [radians] */
    double velocity_ = 0;   /* [radians/s] */
    double torque_ = 0;     /* [Nm] */
    double temperature_ = 0;   /* [Celcius] */
    bool fault = false;
};

struct ControllerParameters
{
    double position_max_ = 0;
    double position_min_ = 0;
    double position_offset_ = 0;
    double velocity_max_ = 0;
    double torque_max_ = 0;
    int direction_ = 1;
    int id_ = 0;             /* Usage in your bridge (check moteus bridge)*/
    int bus_ = 0;            /* Usage in your bridge (check moteus bridge)*/
};

}

#endif