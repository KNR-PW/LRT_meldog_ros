#include "../include/joint_actuator_transform/Transmissions.hpp"

using namespace joint_actuator_transform;

SimpleTransmission::SimpleTransmission(size_t joint_index, double reduction):
joint_index_(joint_index), reduction_(reduction) {}

void SimpleTransmission::calculate_transmission_triplets(std::vector<T>& triplet_vector)
{
    /* 
        Simple q_motor = reduction * q_joint 
        A[joint_index, joint_index] = reduction
    */
    triplet_vector.push_back(T(joint_index_, joint_index_, reduction_)); 
}


FourBarLinkageTransmission::FourBarLinkageTransmission(const std::vector<size_t>& joint_indexes, 
        std::vector<double> actuator_reductions, std::vector<double> joint_reductions):
        joint_indexes_(joint_indexes), actuator_reductions_(actuator_reductions), 
        joint_reductions_(joint_reductions) {}

void FourBarLinkageTransmission::calculate_transmission_triplets(std::vector<T>& triplet_vector)
{
    /* 
        q_motor_1 = a_reduction_1 * j_reduction_1 * q_joint_1
        q_motor_2 = a_reduction_2 * q_joint_1 + a_reduction_2 * j_reduction_2 * q_joint_2
        A[joint_index_1, joint_index_1] = a_reduction_1 * j_reduction_1
        A[joint_index_2, joint_index_1] = a_reduction_2
        A[joint_index_2, joint_index_2] = a_reduction_2 * j_reduction_2
    */
    triplet_vector.push_back(T(joint_indexes_[0], joint_indexes_[0], 
    actuator_reductions_[0]*joint_reductions_[0]));

    triplet_vector.push_back(T(joint_indexes_[1], joint_indexes_[0], 
    actuator_reductions_[1]));

    triplet_vector.push_back(T(joint_indexes_[1], joint_indexes_[1], 
    actuator_reductions_[1]*joint_reductions_[1]));
}