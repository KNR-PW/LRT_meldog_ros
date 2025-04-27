# Joint command message
#
# This message contains the name of controlled joint, feed-forward torque,
# desired joint position and velocity together with the kp_scale and kd_scale
# constants for a PD controller.
#
# Applied joint torque is calculated as follows:
# effort = feedforward_effort + kp_scale * kp * position_error 
# + kd_scale * kd * velocity_error + ki * integration_error 
# where kp, kd and ki are constants in joint controller parameters

std_msgs/Header header

string[] name
float64[] desired_position
float64[] desired_velocity
float64[] kp_scale
float64[] kd_scale
float64[] feedforward_effort

