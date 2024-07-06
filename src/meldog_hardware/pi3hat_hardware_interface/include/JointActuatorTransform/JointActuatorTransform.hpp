#ifndef _JOINT_ACTUATOR_TRANSFORM_
#define _JOINT_ACTUATOR_TRANSFORM_

#include <vector>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
class JointActuatorTransform
{
    private:
    std::vector<MotorState>& motor_states_;
    std::vector<JointState>& joint_states_; // Dodaj strukture
    public:

    JointActuatorTransform();
    void make_transform_matrix(std::vector<double> gearbox_ratios, std::vector<Kinematic_Relations>& relations); // Dodaj klase do relacji między jointami
    void to_joint_space();
    void to_actuator_space();
};


#endif

// TODO:
// STATE DLA JOINTA I SILNIKA, KLASA RELACJI I GEARBOXA, TWORZENIE MACIERZY TRANSFORMACJI W EIGENIE