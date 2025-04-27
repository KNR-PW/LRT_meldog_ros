#ifndef JOINT_CONTROLLER_INTERFACE_HPP_
#define JOINT_CONTROLLER_INTERFACE_HPP_

#include <memory>
#include <string>
#include <vector>
#include <functional>

#include "hardware_interface/types/hardware_interface_type_values.hpp"

#include "joint_controller_msgs/msg/joint_command.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "controller_interface/chainable_controller_interface.hpp"
#include "controller_interface/controller_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "realtime_tools/realtime_buffer.h"
#include "realtime_tools/realtime_publisher.h"

#include "joint_controller_ros2_control/joint_controller_core/joint_controller_core.hpp"
#include "joint_controller_ros2_control/visibility_control.hpp"
#include "joint_controller_parameters.hpp"

namespace joint_controller
{

  class JointController : public controller_interface::ChainableControllerInterface
  {

    public:

    JOINT_CONTROLLER_INTERFACE__VISIBILITY_PUBLIC 
    JointController();

    JOINT_CONTROLLER_INTERFACE__VISIBILITY_PUBLIC 
    controller_interface::CallbackReturn on_init() override;

    JOINT_CONTROLLER_INTERFACE__VISIBILITY_PUBLIC 
    controller_interface::InterfaceConfiguration command_interface_configuration() const override;

    JOINT_CONTROLLER_INTERFACE__VISIBILITY_PUBLIC 
    controller_interface::InterfaceConfiguration state_interface_configuration() const override;

    JOINT_CONTROLLER_INTERFACE__VISIBILITY_PUBLIC 
    controller_interface::CallbackReturn on_cleanup(
      const rclcpp_lifecycle::State & previous_state) override;

    JOINT_CONTROLLER_INTERFACE__VISIBILITY_PUBLIC 
    controller_interface::CallbackReturn on_configure(
      const rclcpp_lifecycle::State & previous_state) override;

    JOINT_CONTROLLER_INTERFACE__VISIBILITY_PUBLIC 
    controller_interface::CallbackReturn on_activate(
      const rclcpp_lifecycle::State & previous_state) override;

    JOINT_CONTROLLER_INTERFACE__VISIBILITY_PUBLIC 
    controller_interface::CallbackReturn on_deactivate(
      const rclcpp_lifecycle::State & previous_state) override;

    #ifdef ROS2_CONTROL_VER_3
    JOINT_CONTROLLER_INTERFACE__VISIBILITY_PUBLIC
    controller_interface::return_type update_reference_from_subscribers(
      const rclcpp::Time & time, const rclcpp::Duration & period) override;
    #else
    JOINT_CONTROLLER_INTERFACE__VISIBILITY_PUBLIC
    controller_interface::return_type update_reference_from_subscribers() override; 
    #endif

    JOINT_CONTROLLER_INTERFACE__VISIBILITY_PUBLIC 
    controller_interface::return_type update_and_write_commands(
      const rclcpp::Time & time, const rclcpp::Duration & period) override;

    protected:
    std::shared_ptr<joint_controller::ParamListener> param_listener_; // TODO DODAJ TO
    joint_controller::Params params_;
    
    std::vector<hardware_interface::CommandInterface> on_export_reference_interfaces() override;

    #ifdef ROS2_CONTROL_VER_3
    std::vector<hardware_interface::StateInterface> on_export_state_interfaces() override; 
    #endif

    bool on_set_chained_mode(bool chained_mode) override;

    private:

    /* Reset JointCommand message */
    using JointCommandMsg = joint_controller_msgs::msg::JointCommand;
    void reset_controller_reference_msg(
        const std::shared_ptr<JointCommandMsg>& _msg, const std::vector<std::string> & _joint_names);

    /* Get state and command interfaces from hardware interface */
    controller_interface::CallbackReturn sort_state_interfaces();
    controller_interface::CallbackReturn sort_command_interfaces();

    /* Get parameters for joints and setup JointControllerCore objects */
    controller_interface::CallbackReturn configure_joints();

    /* Callback for subscriber */
    void reference_callback(const std::shared_ptr<JointCommandMsg> _reference_msg);

    /* Number of joints */
    size_t joint_num_;

    /* Joint names */
    std::vector<std::string> joint_names_;

    /* Joint commands and states */
    using JointCommands = joint_controller_core::JointCommands;
    using JointStates = joint_controller_core::JointStates;
    std::vector<JointCommands> joint_commands_;
    std::vector<JointStates> joint_states_;

    /* For checking if velocity and feedforward effort reference interfaces are used */
    bool has_position_interface_ = false;
    bool has_velocity_interface_ = false;
    bool has_feedforward_effort_interface_ = false;

    /* For checking if kp and kd reference interfaces are used */
    bool has_kp_scale_interface_ = false;
    bool has_kd_scale_interface_ = false;

    /* Controller implementation objects */
    using JointControllerCore = joint_controller_core::JointControllerCore;
    std::vector<JointControllerCore> joint_controllers_;
    
    /* Realtime subscriber */
    rclcpp::Subscription<JointCommandMsg>::SharedPtr command_subscriber_ = nullptr;
    realtime_tools::RealtimeBuffer<std::shared_ptr<JointCommandMsg>> input_commands_;

    /* Default interfaces */
    std::vector<std::string> default_state_interfaces_{"position", "velocity"};
    std::vector<std::string> default_command_interface_{"effort"};

    /* All loaned interfaces from hardware interface */
    using loaned_command_interfaces_t = std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>>;
    using loaned_state_interfaces_t = std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>>;
    loaned_command_interfaces_t effort_command_interfaces_;
    loaned_state_interfaces_t position_state_interfaces_;
    loaned_state_interfaces_t velocity_state_interfaces_;

  };
};  

#endif  