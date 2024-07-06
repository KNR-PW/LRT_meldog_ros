#ifndef _JOINT_ACTUATOR_TRANSFORM_
#define _JOINT_ACTUATOR_TRANSFORM_

#include <vector>
#include <memory>
#include "Transmissions.hpp"

namespace joint_actuator_transform
{

    class JointActuatorTransform
    {
        private:

        using vector = Eigen::Vector<double, Eigen::Dynamic>;
        using TransmissionVector = std::vector<std::unique_ptr<Transmission>>;

        /* vectors of joint states */
        vector& joint_positions_;
        vector& joint_velocities_;
        vector& joint_torques_;

        /* vetors of actuator states */
        vector& actuator_positions_;
        vector& actuator_velocities_;
        vector& actuator_torques_;
        /* actuator -> joint transform matrixes */
        Eigen::SparseMatrix<double> joint_position_velocity_matrix_;
        Eigen::SparseMatrix<double> joint_torque_matrix_;
        /* joint -> actuator matrixes */
        Eigen::SparseMatrix<double> actuator_position_velocity_matrix_;
        Eigen::SparseMatrix<double> actuator_torque_matrix_;

        public:
        /* Constructor */
        JointActuatorTransform(vector joint_positions, vector& joint_velocities, vector& joint_torques, 
        vector& actuator_positions, vector& actuator_velocities, vector& actuator_torques);

        /* Create all transform matrixes */
        void make_transform_matrixes(const TransmissionVector& transmission_vector);

        /* joint <-> actuator transformations */
        void positions_to_joint_space();
        void positions_to_actuator_space();
        void velocities_to_joint_space();
        void velocities_to_actuator_space();
        void torques_to_joint_space();
        void torques_to_actuator_space();
    };
};

#endif
