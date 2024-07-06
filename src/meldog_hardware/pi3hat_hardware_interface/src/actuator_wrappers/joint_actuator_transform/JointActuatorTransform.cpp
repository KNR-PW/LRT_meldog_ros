#include "../include/joint_actuator_transform/JointActuatorTransform.hpp"


using namespace joint_actuator_transform;

JointActuatorTransform::JointActuatorTransform(vector joint_positions, vector& joint_velocities, vector& joint_torques, 
        vector& actuator_positions, vector& actuator_velocities, vector& actuator_torques):
        joint_positions_(joint_positions), joint_velocities_(joint_velocities),
        joint_torques_(joint_torques), actuator_positions_(actuator_positions),
        actuator_velocities_(actuator_velocities), actuator_torques_(actuator_torques) {}


void JointActuatorTransform::make_transform_matrixes(const TransmissionVector& transmission_vector)
{
    /*  Create sparse matrix for transforming joint position and velocities to actuator space:
        - Q_actuator = A * Q_joint
        - V_actuator = A * V_joint
    */
    std::vector<Eigen::Triplet<double>> transmission_triplets;
    for(const auto& transmission: transmission_vector)
    {
        transmission->calculate_transmission_triplets(transmission_triplets);
    }
    actuator_position_velocity_matrix_.setFromTriplets(transmission_triplets.begin(), transmission_triplets.end());

    /* Prepare solvers for calculating other sparse matrixes*/
    Eigen::SimplicialLDLT<Eigen::SparseMatrix<double>> solver;
    Eigen::SparseMatrix<double> I(actuator_position_velocity_matrix_.rows(),actuator_position_velocity_matrix_.cols());
    I.setIdentity();

    /*  Create sparse matrix for transforming actuator position and velocities to joint space:
        - Q_joint = B * Q_actuator 
        - V_joint = B * V_actuator 
        - B = (A)^(-1)
    */
    solver.compute(actuator_position_velocity_matrix_);
    joint_position_velocity_matrix_ = solver.solve(I);

    /*  Create sparse matrix for transforming actuator torques to joint space:
        - Tau_joint = C * Tau_actuator 
        - C = (A)^transpose
    */
    joint_torque_matrix_ = actuator_position_velocity_matrix_.transpose();

    /*  Create sparse matrix for transforming joint torques to actuator space:
        - Tau_actuator = D * Tau_joint
        - D = (C)^(-1)
    */
    solver.compute(joint_torque_matrix_);
    actuator_torque_matrix_ = solver.solve(I);

    /* Compressing all sparse matrixes */
    actuator_position_velocity_matrix_.makeCompressed();
    actuator_torque_matrix_.makeCompressed();
    joint_position_velocity_matrix_.makeCompressed();
    joint_torque_matrix_.makeCompressed();
}

void JointActuatorTransform:: positions_to_joint_space()
{
    joint_positions_ = joint_position_velocity_matrix_ * actuator_positions_;
}

void JointActuatorTransform:: positions_to_actuator_space()
{
    actuator_positions_ = actuator_position_velocity_matrix_ * joint_positions_;
}

void JointActuatorTransform:: velocities_to_joint_space()
{
    joint_velocities_ = joint_position_velocity_matrix_ * actuator_velocities_;
}

void JointActuatorTransform:: velocities_to_actuator_space()
{
    actuator_velocities_= actuator_position_velocity_matrix_ * joint_velocities_;
}

void JointActuatorTransform:: torques_to_joint_space()
{
    joint_torques_ = joint_torque_matrix_ * actuator_torques_;
}
void JointActuatorTransform:: torques_to_actuator_space()
{
    actuator_torques_ = actuator_torque_matrix_ * joint_torques_;
}