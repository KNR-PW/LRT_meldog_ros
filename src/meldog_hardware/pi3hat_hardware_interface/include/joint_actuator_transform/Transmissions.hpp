#ifndef _TRANSMISSIONS_
#define _TRANSMISSIONS_

#include <vector>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Sparse>


namespace joint_actuator_transform
{
    class Transmission
    {
        public:
        using T = Eigen::Triplet<double>;

        /* Calculates new triplets for sparse transmission matrix */
        virtual void calculate_transmission_triplets(std::vector<T>& triplet_vector) = 0;
        virtual ~Transmission() = default;
    };

    /*  Simple transmission (gearboxes, simple pulley systems etc.):
        http://docs.ros.org/en/jade/api/transmission_interface/html/c++/classtransmission__interface_1_1SimpleTransmission.html
    */
    class SimpleTransmission: Transmission
    {
        private:
        size_t joint_index_;
        double reduction_;

        public:
        SimpleTransmission(size_t joint_index, double reduction);
        void calculate_transmission_triplets(std::vector<T>& triplet_vector) override;

    };

    /*  More complicated systems where second actuator is coupled with first one, like shown here:
        http://docs.ros.org/en/jade/api/transmission_interface/html/c++/classtransmission__interface_1_1FourBarLinkageTransmission.html
    */
    class FourBarLinkageTransmission: Transmission
    {
        private:
        std::vector<size_t> joint_indexes_;
        std::vector<double> actuator_reductions_;
        std::vector<double> joint_reductions_;

        public:
        FourBarLinkageTransmission(const std::vector<size_t>& joint_indexes, 
        std::vector<double> actuator_reductions, std::vector<double> joint_reductions);
        void calculate_transmission_triplets(std::vector<T>& triplet_vector) override;

    };
};


#endif