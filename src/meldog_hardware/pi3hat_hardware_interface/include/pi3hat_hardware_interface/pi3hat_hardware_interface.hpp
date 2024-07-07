#ifndef PI3HAT_HARDWARE_INTERFACE__PI3HAT_HARDWARE_INTERFACE_
#define PI3HAT_HARDWARE_INTERFACE__PI3HAT_HARDWARE_INTERFACE_


#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "actuator_wrappers/MoteusWrapper.hpp"
#include "joint_actuator_transform/JointActuatorTransform.hpp"

#include "pi3hat/pi3hat.h"
#include "pi3hat/realtime.h"

#include "visibility_control.hpp"

namespace pi3hat_hardware_interface
{
    class Pi3HatHardwareInterface : public hardware_interface::SystemInterface
    {
    public:
        RCLCPP_SHARED_PTR_DEFINITIONS(Pi3HatHardwareInterface)

        ROS2_CONTROL_PI3HAT_HARDWARE_PUBLIC
        hardware_interface::CallbackReturn on_init(
            const hardware_interface::HardwareInfo &info) override;

        ROS2_CONTROL_PI3HAT_HARDWARE_PUBLIC
        hardware_interface::CallbackReturn on_configure(
            const rclcpp_lifecycle::State &previous_state) override;

        ROS2_CONTROL_PI3HAT_HARDWARE_PUBLIC
        std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
        
        ROS2_CONTROL_PI3HAT_HARDWARE_PUBLIC
        std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

        ROS2_CONTROL_PI3HAT_HARDWARE_PUBLIC
        hardware_interface::CallbackReturn on_activate(
            const rclcpp_lifecycle::State &previous_state) override;

        ROS2_CONTROL_PI3HAT_HARDWARE_PUBLIC
        hardware_interface::CallbackReturn on_deactivate(
            const rclcpp_lifecycle::State &previous_state) override;

        ROS2_CONTROL_PI3HAT_HARDWARE_PUBLIC
        hardware_interface::return_type read(
            const rclcpp::Time &time, const rclcpp::Duration &period) override;

        ROS2_CONTROL_PI3HAT_HARDWARE_PUBLIC
        hardware_interface::return_type write(
            const rclcpp::Time &time, const rclcpp::Duration &period) override;

    private:

        mjbots::pi3hat::Pi3Hat& pi3hat_;
        mjbots::pi3hat::Pi3Hat::Input pi3hat_input_;
        mjbots::pi3hat::Attitude attitude_;
        std::vector<mjbots::pi3hat::CanFrame> tx_can_frames_; //Pomysl jeszcze nad tym!
        std::vector<mjbots::pi3hat::CanFrame> rx_can_frames_;

        /* IMU states */ 
        std::array<double, 4> hw_state_imu_orientation_;         // x, y, z, w
        std::array<double, 3> hw_state_imu_angular_velocity_;    // x, y, z
        std::array<double, 3> hw_state_imu_linear_acceleration_; // x, y, z

        /* Actuator CAN config */
        std::vector<int> hw_actuator_can_channels_;
        std::vector<int> hw_actuator_can_ids_;

        /* Actuator parameters */
        std::vector<double> hw_actuator_position_scales_;
        std::vector<double> hw_actuator_velocity_scales_;
        std::vector<double> hw_actuator_effort_scales_;
        std::vector<double> hw_actuator_kp_scales_;
        std::vector<double> hw_actuator_kd_scales_;
        std::vector<int> hw_actuator_axis_directions_;
        std::vector<double> hw_actuator_position_offsets_;

        // Actuator limits
        std::vector<double> hw_actuator_position_mins_; 
        std::vector<double> hw_actuator_position_maxs_;
        std::vector<double> hw_actuator_velocity_maxs_;
        std::vector<double> hw_actuator_effort_maxs_;
        std::vector<double> hw_actuator_kp_maxs_;
        std::vector<double> hw_actuator_kd_maxs_;

        using eigen_vector = Eigen::Vector<double, Eigen::Dynamic>;

        /* Joint states */
        eigen_vector hw_joint_positions_;
        eigen_vector hw_joint_velocities_;
        eigen_vector hw_joint_torques_;

        // Actuator commands
        eigen_vector hw_command_positions_;
        eigen_vector hw_command_velocities_;
        eigen_vector hw_command_torques_;
    
    };

} 

#endif 
