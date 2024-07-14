#include "../../include/imu/IMUTransform.hpp"


using namespace IMU;

void IMUTransform::transform_attitude(mjbots::pi3hat::Attitude& attitude)
{
    /* Comment taken from: https://github.com/G-Levine/pi3hat_hardware_interface/blob/main/src/pi3hat_hardware_interface.cpp */ 
         /* 
            The quaternion returned by the pi3hat is the rotation from the gravity frame to the IMU frame.
                Gravity frame:
                    +x and +y are parallel to the ground
                    +z points towards the ground
                IMU frame:
                    +x points towards the side of the Pi with the USB-C port
                    +y points towards the side of the Pi with the USB-A ports
                    +z points up from the Pi
            However, we want the rotation from the world frame to the IMU frame.
                World frame:
                    +x and +y are parallel to the ground
                    +z points towards the sky
            Therefore, we need to rotate the quaternion returned by the pi3hat by 180 degrees about the x-axis or y-axis. We choose to rotate about the x-axis.
            Let the quaternion returned by the pi3hat be (x, y, z, w).
            After applying a 180-degree rotation about the x-axis, the new quaternion is:
                (w, -z, y, -x)
         */
    
    double temp_x = attitude.attitude.x;
    double temp_y = attitude.attitude.y;

    attitude.attitude.x = attitude.attitude.w;
    attitude.attitude.w = -temp_x;

    attitude.attitude.y = -attitude.attitude.z;
    attitude.attitude.z = temp_y;
}   