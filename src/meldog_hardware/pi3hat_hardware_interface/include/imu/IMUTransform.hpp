#ifndef _IMU_
#define _IMU_

#include <array>
#include "../pi3hat/pi3hat.h"
#include <algorithm>

namespace IMU
{
    class IMUTransform
    {
        public:

        /* Function for transforming transforming IMU data */
        void transform_attitude(mjbots::pi3hat::Attitude& attitude);
    };

};


#endif