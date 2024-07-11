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

        size_t number_of_actuators;

        std::shared_ptr<mjbots::pi3hat::Pi3Hat> pi3hat_;
        mjbots::pi3hat::Pi3Hat::Input pi3hat_input_;
        mjbots::pi3hat::Attitude attitude_;

        std::shared_ptr<mjbots::pi3hat::CanFrame[]> tx_can_frames_; //Pomysl jeszcze nad tym!
        std::shared_ptr<mjbots::pi3hat::CanFrame[]> rx_can_frames_;

        /* IMU states */ 
        std::array<double, 4> hw_state_imu_orientation_;         // x, y, z, w
        std::array<double, 3> hw_state_imu_angular_velocity_;    // x, y, z
        std::array<double, 3> hw_state_imu_linear_acceleration_; // x, y, z

        /* Actuator CAN config */
        std::vector<int> hw_actuator_can_buses_;  
        std::vector<int> hw_actuator_can_ids_;

        /* Actuator parameters */
        std::vector<double> hw_actuator_position_offsets_; // To wrzucimy do opcji

        /* Actuator limits */
        std::vector<double> hw_actuator_position_mins_; 
        std::vector<double> hw_actuator_position_maxs_; // To wrzucimy do opcji
        std::vector<double> hw_actuator_velocity_maxs_;
        std::vector<double> hw_actuator_effort_maxs_;

        /* Actuator states */
        std::vector<actuator_wrappers::ActuatorState> hw_actuator_states_;

        // Actuator commands
        std::vector<actuator_wrappers::ActuatorCommand> hw_actuator_commands_;

        /* Actuator Wrappers (here change to your own wrapper) */
        std::vector<actuator_wrappers::MoteusWrapper> moteus_wrappers;

        template<class Wrapper>
        void make_commands(std::vector<actuator_wrappers::ActuatorWrapperBase<Wrapper>> actuator_wrappers)
        {
            // TODO: Uporządkuj wcześniej silniki względem id
            for(size_t i = 0; i < number_of_actuators; i++)
            {   
                size_t actuator_id = tx_can_frames_[i].id;
                actuator_wrappers[actuator_id].command_to_tx_frame(tx_can_frames_[i], hw_actuator_commands_[actuator_id]);
            }
        };

        template<class Wrapper>
        void get_states(std::vector<actuator_wrappers::ActuatorWrapperBase<Wrapper>> actuator_wrappers)
        {
            for(size_t i = 0; i < number_of_actuators; i++)
            {   
                size_t actuator_id = tx_can_frames_[i].id;
                actuator_wrappers[actuator_id].rx_frame_to_state(rx_can_frames_[i], hw_actuator_states_[actuator_id]);
            }
        }
    };

} 

#endif 
