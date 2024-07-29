#ifndef _IMU_
#define _IMU_

#include <array>
#include "../3rd_libs/pi3hat/pi3hat.h"
#include <algorithm>
#include <cmath>

namespace IMU
{
    class IMUTransform
    {
        private:
        constexpr static double degrees_to_radians = 2 * M_PI / 360.0;

        public:

        /* Function for transforming transforming IMU data */
        void transform_attitude(mjbots::pi3hat::Attitude& attitude);
    };

};


#endif