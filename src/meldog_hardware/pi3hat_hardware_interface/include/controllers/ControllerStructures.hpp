#ifndef _CONTROLLER_STRUCTURES_HPP_
#define _CONTROLLER_STRUCTURES_HPP_

namespace controller_interface
{

/* Structure for basic actuator command */
struct ControllerCommand
{
    double position_;   /* [radians] */
    double velocity_;   /* [radians/s] */
    double torque_;     /* [Nm] */
};

/* Structure for basic actuator state */
struct ControllerState
{
    double position_;   /* [radians] */
    double velocity_;   /* [radians/s] */
    double torque_;     /* [Nm] */
    double temperature_;   /* [Celcius] */
    bool fault = false;
};

struct ControllerParameters
{
    double position_max_;
    double position_min_;
    double position_offset_;
    double velocity_max_;
    double torque_max_;
    int direction_ = 1;
    int id_;             /* Usage in your bridge (check moteus bridge)*/
    int bus_;            /* Usage in your bridge (check moteus bridge)*/
};

}

#endif