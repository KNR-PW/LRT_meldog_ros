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

#include "motor_wrappers/MoteusWrapper.hpp"

#include "pi3hat/pi3hat.h"
#include "pi3hat/realtime.h"

#include "visibility_control.hpp"

#include <cmath>

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

        std::shared_ptr<mjbots::pi3hat::Pi3Hat> pi3hat_;
        mjbots::pi3hat::Pi3Hat::Input pi3hat_input_;
        mjbots::pi3hat::Attitude attitude_;

        std::shared_ptr<mjbots::pi3hat::CanFrame[]> tx_can_frames_; //Pomysl jeszcze nad tym!
        std::shared_ptr<mjbots::pi3hat::CanFrame[]> rx_can_frames_;

        /* IMU states */ 
        std::array<double, 4> hw_state_imu_orientation_;         // x, y, z, w
        std::array<double, 3> hw_state_imu_angular_velocity_;    // x, y, z
        std::array<double, 3> hw_state_imu_linear_acceleration_; // x, y, z

        /* Motor CAN config */
        std::vector<int> hw_motor_can_buses_;
        std::vector<int> hw_motor_can_ids_;

        /* Motor parameters */
        std::vector<double> hw_motor_position_offsets_;

        /* Motor limits */
        std::vector<double> hw_motor_position_mins_; 
        std::vector<double> hw_motor_position_maxs_;
        std::vector<double> hw_motor_velocity_maxs_;
        std::vector<double> hw_motor_effort_maxs_;

        /* Motor states */
        std::vector<motor_wrappers::MotorState> hw_motor_states_;

        // Motor commands
        std::vector<motor_wrappers::MotorState> hw_motor_commands_;

        /* Motor Wrappers (here change to your own wrapper) */
        std::vector<motor_wrappers::MoteusWrapper> moteus_wrappers;

        template<class Wrapper>
        void make_commands(std::vector<motor_wrappers::MotorWrapperBase<Wrapper>> motor_wrappers)
        {
            for(auto& motor_wrapper: motor_wrappers)
            {
                motor_wrapper.make_command();
            }
        };

        template<class Wrapper>
        void get_states(std::vector<motor_wrappers::MotorWrapperBase<Wrapper>> motor_wrappers)
        {
            for(auto& motor_wrapper: motor_wrappers)
            {
                motor_wrapper.get_state();
            }
        }
    };

} 

#endif 
