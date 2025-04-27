#include "joint_controller_ros2_control/joint_controller.hpp"

using namespace joint_controller;

JointController::JointController(): controller_interface::ChainableControllerInterface(){}
controller_interface::CallbackReturn JointController::on_init()
{
    /* Get parameter listener and parameters structure */
    try
    {
        param_listener_ = std::make_shared<joint_controller::ParamListener>(get_node());
        params_ = param_listener_->get_params();
    }
    catch (const std::exception & e)
    {
        RCLCPP_FATAL(get_node()->get_logger(),
            "Exception thrown during controller's init with message: %s \n", e.what());
        return controller_interface::CallbackReturn::ERROR;
    }
    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration JointController::command_interface_configuration() const
{
    /* Get all loaned command interfaces used by controller from hardware inetrface */
    controller_interface::InterfaceConfiguration command_interfaces_config;
    command_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

    command_interfaces_config.names.reserve(joint_num_);
    for (const auto & joint_name : joint_names_)
    {
        command_interfaces_config.names.push_back(joint_name + "/" + params_.command_interface);
    }
    RCLCPP_INFO(get_node()->get_logger(), "Command interfaces for JointController configured successfully!");
    return command_interfaces_config;
}

controller_interface::InterfaceConfiguration JointController::state_interface_configuration() const
{
    /* Get all loaned state interfaces used by controller from hardware inetrface */
    controller_interface::InterfaceConfiguration state_interfaces_config;

    state_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
    state_interfaces_config.names.reserve(joint_num_ * params_.state_interfaces.size());
    for (const auto & interface : params_.state_interfaces)
    {
        for (const auto & joint_name: joint_names_)
        {
            state_interfaces_config.names.push_back(joint_name + "/" + interface);
        }
    }
    RCLCPP_INFO(get_node()->get_logger(), "State interfaces for JointController configured successfully!");
    return state_interfaces_config;
}

controller_interface::CallbackReturn JointController::on_cleanup(
      const rclcpp_lifecycle::State & previous_state)
{
    /* Clean up all controll structures when controller is terminated */
    joint_names_.clear();
    joint_commands_.clear();
    joint_states_.clear();
    joint_controllers_.clear();
    
    RCLCPP_INFO(get_node()->get_logger(), "JointController cleaned successfully!");
    return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn JointController::on_configure(
      const rclcpp_lifecycle::State & previous_state)
{
    /* Creating joint controllers, reference subscriber and buffer for reference messages */
    auto ret = configure_joints();
    if (ret != CallbackReturn::SUCCESS)
    {
        return ret;
    }

    auto subscribers_qos = rclcpp::SystemDefaultsQoS();
    subscribers_qos.keep_last(1);
    subscribers_qos.best_effort();

    command_subscriber_ = get_node()->create_subscription<JointCommandMsg>(
        "~/joint_commands", subscribers_qos,
        std::bind(&JointController::reference_callback, this, std::placeholders::_1));

    std::shared_ptr<JointCommandMsg> msg = std::make_shared<JointCommandMsg>();
    reset_controller_reference_msg(msg, joint_names_);
    input_commands_.writeFromNonRT(msg);

    RCLCPP_INFO(get_node()->get_logger(), "JointController configured successfully!");
    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn JointController::on_activate(
      const rclcpp_lifecycle::State & previous_state)
{
    /* Add all loaned state and command interfaces to sorted vectors */
    if(sort_state_interfaces() != controller_interface::CallbackReturn::SUCCESS)
    {
        return controller_interface::CallbackReturn::FAILURE;
    }

    if(sort_command_interfaces() != controller_interface::CallbackReturn::SUCCESS)
    {
        return controller_interface::CallbackReturn::FAILURE;
    }

    reset_controller_reference_msg(*(input_commands_.readFromRT()), joint_names_);

    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn JointController::on_deactivate(
    const rclcpp_lifecycle::State & previous_state)
{
    /* Remove all loaned command and state interfaces from structures and release them*/
    position_state_interfaces_.clear();
    velocity_state_interfaces_.clear();
    effort_command_interfaces_.clear();
    release_interfaces();
    
    return controller_interface::CallbackReturn::SUCCESS;
}

#ifdef ROS2_CONTROL_VER_3
controller_interface::return_type JointController::update_reference_from_subscribers(
    const rclcpp::Time & time, const rclcpp::Duration & period
)
{
    auto current_ref = input_commands_.readFromRT();
    JointCommandMsg& joint_reference_msg = *(*current_ref);
    
    size_t message_size = joint_reference_msg.name.size();
    for(size_t i = 0; i < message_size; ++i)
    {
        const auto& message_joint_name = joint_reference_msg.name[i];
        auto joint_name_iterator = std::find(joint_names_.begin(), joint_names_.end(), message_joint_name);
        if(joint_name_iterator == joint_names_.end())
        {
            RCLCPP_WARN(get_node()->get_logger(),
                "Joint named (%s) doesn't exist!",
                message_joint_name.c_str());
            continue;
        }

        size_t index = std::distance(joint_names_.begin(), joint_name_iterator);

        if ((!std::isnan(joint_reference_msg.desired_position[index])) 
                && (!std::isnan(joint_reference_msg.desired_velocity[index]))
                && (!std::isnan(joint_reference_msg.kp_scale[index])) 
                && (!std::isnan(joint_reference_msg.kd_scale[index]))
                && (!std::isnan(joint_reference_msg.feedforward_effort[index])))
        {
            JointCommands& joint_command = joint_commands_[index];
            if(has_position_interface_) joint_command.desired_position_ = joint_reference_msg.desired_position[i];
            if(has_velocity_interface_) joint_command.desired_velocity_ = joint_reference_msg.desired_velocity[i];
            if(has_feedforward_effort_interface_) joint_command.feedforward_effort_ = joint_reference_msg.feedforward_effort[i];
            if(has_kp_scale_interface_) joint_command.kp_scale_ = joint_reference_msg.kp_scale[i];
            if(has_kd_scale_interface_) joint_command.kd_scale_ = joint_reference_msg.kd_scale[i];
        }
    }
    return controller_interface::return_type::OK;
}

#else
controller_interface::return_type JointController::update_reference_from_subscribers()
{
    /* Update reference commands from message */
    auto current_ref = input_commands_.readFromRT();
    JointCommandMsg& joint_reference_msg = *(*current_ref);
    size_t message_size = joint_reference_msg.name.size();
    for(size_t i = 0; i < message_size; ++i)
    {
        const auto& message_joint_name = joint_reference_msg.name[i];
        auto joint_name_iterator = std::find(joint_names_.begin(), joint_names_.end(), message_joint_name);
        if(joint_name_iterator == joint_names_.end())
        {
            RCLCPP_WARN(get_node()->get_logger(),
                "Joint named (%s) doesn't exist!",
                message_joint_name.c_str());
            continue;
        }

        size_t index = std::distance(joint_names_.begin(), joint_name_iterator);

        if ((!std::isnan(joint_reference_msg.desired_position[index])) 
                && (!std::isnan(joint_reference_msg.desired_velocity[index]))
                && (!std::isnan(joint_reference_msg.kp_scale[index])) 
                && (!std::isnan(joint_reference_msg.kd_scale[index]))
                && (!std::isnan(joint_reference_msg.feedforward_effort[index])))
        {
            JointCommands& joint_command = joint_commands_[index];
            if(has_position_interface_) joint_command.desired_position_ = joint_reference_msg.desired_position[i];
            if(has_velocity_interface_) joint_command.desired_velocity_ = joint_reference_msg.desired_velocity[i];
            if(has_feedforward_effort_interface_) joint_command.feedforward_effort_ = joint_reference_msg.feedforward_effort[i];
            if(has_kp_scale_interface_) joint_command.kp_scale_ = joint_reference_msg.kp_scale[i];
            if(has_kd_scale_interface_) joint_command.kd_scale_ = joint_reference_msg.kd_scale[i];
        }
    }
    return controller_interface::return_type::OK;
}
#endif

controller_interface::return_type JointController::update_and_write_commands(
      const rclcpp::Time & time, const rclcpp::Duration & period)
{
    /* Get current state and send commands to loaned comannd interfaces */
    for(size_t i = 0; i < joint_num_; ++i)
    {
        joint_states_[i].position_ = position_state_interfaces_[i].get().get_value();
        joint_states_[i].velocity_ = velocity_state_interfaces_[i].get().get_value();

        double torque = joint_controllers_[i].calculateEffort(joint_commands_[i], joint_states_[i]);
        effort_command_interfaces_[i].get().set_value(torque);
    }
    return controller_interface::return_type::OK;
}

bool JointController::on_set_chained_mode(bool chained_mode)
{
    /* Always accept switch to/from chained mode */

    return true || chained_mode;
}

controller_interface::CallbackReturn JointController::configure_joints()
{
    /* Create all joint controllers and important joint parameters */
    if(params_.joint_names.size() != params_.pid_gains.joint_names_map.size())
    {
        RCLCPP_FATAL(get_node()->get_logger(),
            "Size of 'pid gains' (%zu) map and number or 'joint_names' (%zu) have to be the same!",
            params_.pid_gains.joint_names_map.size(), params_.joint_names.size());
        return CallbackReturn::FAILURE;
    }

    if(params_.joint_names.size() != params_.joint_params.joint_names_map.size())
    {
        RCLCPP_FATAL(get_node()->get_logger(),
            "Size of 'joint params' (%zu) map and number or 'joint_names' (%zu) have to be the same!",
            params_.joint_params.joint_names_map.size(), params_.joint_names.size());
        return CallbackReturn::FAILURE;
    }

    if(std::find(params_.reference_interfaces.begin(), params_.reference_interfaces.end(), hardware_interface::HW_IF_POSITION) != params_.reference_interfaces.end())
    {
        has_position_interface_ = true;
    }
    else
    {
        for(auto& joint_command : joint_commands_)
        {
            joint_command.desired_position_= 0.0;
        }
    }

    if(std::find(params_.reference_interfaces.begin(), params_.reference_interfaces.end(), hardware_interface::HW_IF_VELOCITY) != params_.reference_interfaces.end())
    {
        has_velocity_interface_ = true;
    }
    else
    {
        for(auto& joint_command : joint_commands_)
        {
            joint_command.desired_velocity_ = 0.0;
        }
    }

    if(std::find(params_.reference_interfaces.begin(), params_.reference_interfaces.end(), hardware_interface::HW_IF_EFFORT) != params_.reference_interfaces.end())
    {
        has_feedforward_effort_interface_ = true;
    }
    else
    {
        for(auto& joint_command : joint_commands_)
        {
            joint_command.feedforward_effort_ = 0.0;
        }
    }

    if(std::find(params_.reference_interfaces.begin(), params_.reference_interfaces.end(), "kp_scale") != params_.reference_interfaces.end())
    {
        has_kp_scale_interface_ = true;
    }
    else
    {
        for(auto& joint_command : joint_commands_)
        {
            joint_command.kp_scale_ = 1.0;
        }
    }

    if(std::find(params_.reference_interfaces.begin(), params_.reference_interfaces.end(), "kd_scale") != params_.reference_interfaces.end())
    {
        has_kd_scale_interface_ = true;
    }
    else
    {
        for(auto& joint_command : joint_commands_)
        {
            joint_command.kd_scale_ = 1.0;
        }
    }

    joint_num_ = params_.joint_names.size();

    double pid_frequency = params_.frequency;

    for(size_t i = 0; i < joint_num_; ++i)
    {
        joint_names_.push_back(params_.joint_names[i]);
        joint_commands_.push_back(JointCommands());
        joint_states_.push_back(JointStates());


        joint_controller_core::JointParameters joint_params;
        auto listener_joint_param = params_.joint_params.joint_names_map[params_.joint_names[i]];
        joint_params.position_max_ = listener_joint_param.position_max;
        joint_params.position_min_ = listener_joint_param.position_min;
        joint_params.position_offset_ = listener_joint_param.position_offset;
        joint_params.velocity_max_ = listener_joint_param.velocity_max;
        joint_params.effort_max_ = listener_joint_param.effort_max;


        pid_controller::PidParameters pid_params;
        auto listener_pid_param = params_.pid_gains.joint_names_map[params_.joint_names[i]];
        pid_params.proportional_coef_ = listener_pid_param.p;
        pid_params.derivative_coef_ = listener_pid_param.d;
        pid_params.integral_coef_ = listener_pid_param.i;
        pid_params.integration_limit_ = listener_pid_param.ilimit;

        joint_controller_core::JointControllerCore joint_controller(joint_params, 
            pid_params, pid_frequency);
        
        joint_controllers_.push_back(joint_controller);
    }
    return controller_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::CommandInterface> JointController::on_export_reference_interfaces()
{
    /* Export reference interfaces for other controllers to use */
    std::vector<hardware_interface::CommandInterface> reference_interfaces;
    reference_interfaces.reserve(params_.reference_interfaces.size() * params_.joint_names.size());

    for (const auto & interface : params_.reference_interfaces)
    {
        for (size_t i = 0; i < joint_num_; ++i)
        {
            const auto& joint_name = joint_names_[i];
            if(interface == hardware_interface::HW_IF_POSITION)
            {
                reference_interfaces.push_back(hardware_interface::CommandInterface(
                    get_node()->get_name(), joint_name + "/" + interface, &joint_commands_[i].desired_position_));
            }
            else if(interface == hardware_interface::HW_IF_VELOCITY)
            {
                reference_interfaces.push_back(hardware_interface::CommandInterface(
                    get_node()->get_name(), joint_name + "/" + interface, &joint_commands_[i].desired_velocity_));
            }
            else if(interface == hardware_interface::HW_IF_EFFORT)
            {
                reference_interfaces.push_back(hardware_interface::CommandInterface(
                    get_node()->get_name(), joint_name + "/" + interface, &joint_commands_[i].feedforward_effort_));
            }
            else if(interface == "kp_scale")
            {
                reference_interfaces.push_back(hardware_interface::CommandInterface(
                    get_node()->get_name(), joint_name + "/" + interface, &joint_commands_[i].kp_scale_));
            }
            else if(interface == "kd_scale")
            {
                reference_interfaces.push_back(hardware_interface::CommandInterface(
                    get_node()->get_name(), joint_name + "/" + interface, &joint_commands_[i].kd_scale_));
            }
            else
            {
                RCLCPP_FATAL(get_node()->get_logger(),
                    "Exception thrown during controller's reference export, not suppossed to happen lol\n");
            }
        }
    }
    reference_interfaces_.resize(joint_names_.size() * params_.reference_interfaces.size(), std::numeric_limits<double>::quiet_NaN());
    return reference_interfaces;
}
#ifdef ROS2_CONTROL_VER_3
std::vector<hardware_interface::StateInterface> JointController::on_export_state_interfaces() 
{
    /* No state interfaces exported, it is a one way controller! */
    std::vector<hardware_interface::StateInterface> state_interfaces;
    return state_interfaces;
}
#endif

void JointController::reset_controller_reference_msg(
  const std::shared_ptr<JointCommandMsg>& _msg, const std::vector<std::string> & _joint_names)
{
    /* Reset all values in message */
    _msg->name = _joint_names;
    _msg->header.frame_id = "";
    _msg->header.stamp.sec = 0;
    _msg->header.stamp.nanosec = 0;
    _msg->desired_position.resize(_joint_names.size(), std::numeric_limits<double>::quiet_NaN());
    _msg->desired_velocity.resize(_joint_names.size(), std::numeric_limits<double>::quiet_NaN());
    _msg->kp_scale.resize(_joint_names.size(), 1);
    _msg->kd_scale.resize(_joint_names.size(), 1);
    _msg->feedforward_effort.resize(_joint_names.size(), std::numeric_limits<double>::quiet_NaN());
}

controller_interface::CallbackReturn JointController::sort_state_interfaces()
{
    /* Add all loaned state interfaces to internal controller structures */
    if (state_interfaces_.empty())
    {
        return controller_interface::CallbackReturn::ERROR;
    }
    for (size_t i = 0; i < joint_num_; ++i)
    {
        std::vector<size_t> position_interface_indexes;
        std::vector<size_t> velocity_interface_indexes;
        for (auto state_interface_iterator = state_interfaces_.begin();
            state_interface_iterator != state_interfaces_.end(); state_interface_iterator++)
        {
            const auto& joint_name = state_interface_iterator->get_prefix_name();
            auto interface_name = state_interface_iterator->get_interface_name();
            size_t index = std::distance(state_interfaces_.begin(), state_interface_iterator);
            if(joint_names_[i] == joint_name)
            {
                if(interface_name == hardware_interface::HW_IF_POSITION)
                {
                    position_interface_indexes.push_back(index);
                }
                else if(interface_name == hardware_interface::HW_IF_VELOCITY)
                {
                    velocity_interface_indexes.push_back(index);
                }
            }
        }
        for(auto position_interface_index: position_interface_indexes)
        {
            position_state_interfaces_.push_back(state_interfaces_[position_interface_index]);
        }
        for(auto velocity_interface_index: velocity_interface_indexes)
        {
            velocity_state_interfaces_.push_back(state_interfaces_[velocity_interface_index]);
        }
    }


    if(position_state_interfaces_.size() != joint_num_)
    {
        RCLCPP_WARN(get_node()->get_logger(),
            "Size of position state inetrface vector is (%zu), but should be (%zu)",
            effort_command_interfaces_.size(), joint_num_);
        return controller_interface::CallbackReturn::ERROR;
    }
    if(velocity_state_interfaces_.size() != joint_num_)
    {
        RCLCPP_WARN(get_node()->get_logger(),
            "Size of position state inetrface vector is (%zu), but should be (%zu)",
            effort_command_interfaces_.size(), joint_num_);
        return controller_interface::CallbackReturn::ERROR;
    }

    RCLCPP_INFO(get_node()->get_logger(), "State interfaces for JointController sorted successfully!");
    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn JointController::sort_command_interfaces()
{
    /* Add all loaned command interfaces to internal controller structures */
    if (command_interfaces_.empty())
    {
        return controller_interface::CallbackReturn::ERROR;
    }

    for (size_t i = 0; i < joint_num_; ++i)
    {
        std::vector<size_t> effort_interface_indexes;
        for (auto command_interface_iterator = command_interfaces_.begin();
            command_interface_iterator != command_interfaces_.end(); command_interface_iterator++)
        {
            auto joint_name = command_interface_iterator->get_prefix_name();
            auto interface_name = command_interface_iterator->get_interface_name();
            size_t index = std::distance(command_interfaces_.begin(), command_interface_iterator);
            if(joint_names_[i] == joint_name)
            {
                if(interface_name == hardware_interface::HW_IF_EFFORT)
                {
                    effort_interface_indexes.push_back(index);
                }
            }
        }
        for(auto effort_interface_index: effort_interface_indexes)
        {
            effort_command_interfaces_.push_back(command_interfaces_[effort_interface_index]);
        }
    }
    if(effort_command_interfaces_.size() != joint_num_)
    {
        RCLCPP_WARN(get_node()->get_logger(),
            "Size of effort command inetrface map is (%zu), but should be (%zu)",
            effort_command_interfaces_.size(), joint_num_);
        return controller_interface::CallbackReturn::ERROR;
    }

    RCLCPP_INFO(get_node()->get_logger(), "Command interfaces for JointController sorted successfully!");
    return controller_interface::CallbackReturn::SUCCESS;
}

void JointController::reference_callback(const std::shared_ptr<JointCommandMsg> _reference_msg)
{
    /* Callback for subscriber */
    if(_reference_msg->desired_position.size() != joint_num_
        || _reference_msg->desired_velocity.size() != joint_num_
        || _reference_msg->kp_scale.size() != joint_num_
        || _reference_msg->kd_scale.size() != joint_num_
        || _reference_msg->feedforward_effort.size() != joint_num_)
    {
        RCLCPP_WARN(
            get_node()->get_logger(),
            "Size of input joint names (%zu) and/or values is not matching the expected size (%zu).",
            _reference_msg->name.size(), joint_names_.size());
            return;
    }

    if(_reference_msg->name.empty())
    {
        RCLCPP_WARN(
            get_node()->get_logger(),
            "Command massage does not have joint names defined. "
            "Assuming that values have order as defined joint names");
        _reference_msg->name = joint_names_;
    }

    input_commands_.writeFromNonRT(_reference_msg);
}

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(joint_controller::JointController, controller_interface::ChainableControllerInterface)