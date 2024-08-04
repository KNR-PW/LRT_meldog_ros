#include "pi3hat_hardware_interface/pi3hat_hardware_interface.hpp"
//#include "../include/pi3hat_hardware_interface/pi3hat_hardware_interface.hpp"

using namespace pi3hat_hardware_interface;
using namespace controller_interface;

/* MAIN FUNCTIONS */

hardware_interface::CallbackReturn Pi3HatHardwareInterface::on_init(const hardware_interface::HardwareInfo &info)
{
    if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS)
    {
        return hardware_interface::CallbackReturn::ERROR;
    }

    logger_ = std::make_unique<rclcpp::Logger>(
    rclcpp::get_logger("Pi3HatHardwareInterface"));

    joint_controller_number_ = info_.joints.size();

    controller_commands_.resize(joint_controller_number_);
    controller_states_.resize(joint_controller_number_);
    controller_transmission_passthrough_.resize(joint_controller_number_);


    joint_commands_.resize(joint_controller_number_);
    joint_states_.resize(joint_controller_number_);
    joint_transmission_passthrough_.resize(joint_controller_number_);


    /* Prepare controller bridges */
    for (const hardware_interface::ComponentInfo &joint : info_.joints)
    {
        controller_interface::ControllerParameters params;
        WrapperType type;
        try
        {
            params = get_controller_parameters(joint);

            std::string type_string = joint.parameters.at("controller_type");
            controller_start_positions_.push_back(std::stod(joint.parameters.at("motor_position_start")));
            type = choose_wrapper_type(type_string);
        }
        catch(const std::exception& e)
        {
            RCLCPP_FATAL(*logger_, "Error reading motor/controller parameters!");
            return hardware_interface::CallbackReturn::ERROR;
        }

        try
        {
            add_controller_bridge(params, type);
        }
        catch(const std::exception& e)
        {
            RCLCPP_FATAL(*logger_, "Error creating motor controller!");
            return hardware_interface::CallbackReturn::ERROR;
        }
    }

    /* Prepare transmissions */
    try
    {
        create_transmission_interface(info_);
    }
    catch(const std::exception& e)
    {
        RCLCPP_FATAL(*logger_, "%s", e.what());
        return hardware_interface::CallbackReturn::ERROR;
    }

    /* Standard CAN config */
    mjbots::pi3hat::Pi3Hat::CanConfiguration can_config;

    /* Configure the Pi3Hat for 1000hz IMU sampling */ 
    mjbots::pi3hat::Pi3Hat::Configuration config;
    config.attitude_rate_hz = 1000;
    
    /* Set the mounting orientation of the IMU */
    try
    {
        config.mounting_deg.yaw = std::stod(info_.hardware_parameters.at("imu_mounting_deg.yaw"));
        config.mounting_deg.pitch = std::stod(info_.hardware_parameters.at("imu_mounting_deg.pitch"));
        config.mounting_deg.roll = std::stod(info_.hardware_parameters.at("imu_mounting_deg.roll"));
    }
    catch(const std::exception& e)
    {
        RCLCPP_FATAL(*logger_, "Error reading IMU parameters!");
        return hardware_interface::CallbackReturn::ERROR;
    }
    
    /* Initialize the Pi3Hat input */ 
    pi3hat_input_ = mjbots::pi3hat::Pi3Hat::Input();
    pi3hat_input_.request_attitude = true;
    pi3hat_input_.wait_for_attitude = true;
    pi3hat_input_.attitude = &attitude_;

    tx_can_frames_.resize(joint_controller_number_);
    rx_can_frames_.resize(joint_controller_number_);
    
    mjbots::pi3hat::Span<mjbots::pi3hat::CanFrame> rx_can_frames_span_(&rx_can_frames_[0], joint_controller_number_); 
    pi3hat_input_.rx_can = rx_can_frames_span_;
    mjbots::pi3hat::Span<mjbots::pi3hat::CanFrame> tx_can_frames_span_(&tx_can_frames_[0], joint_controller_number_); 
    pi3hat_input_.tx_can = tx_can_frames_span_;

    config.can[0] = can_config;
    config.can[1] = can_config;
    config.can[2] = can_config;
    config.can[3] = can_config;
    config.can[4] = can_config;

    /* Initialize the Pi3Hat and realtime options */
    pi3hat_ =  std::make_shared<mjbots::pi3hat::Pi3Hat>(config);
    mjbots::pi3hat::ConfigureRealtime(0);
    
    RCLCPP_INFO(*logger_, "Hardware Interface successfully initialized!");
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn Pi3HatHardwareInterface::on_configure(const rclcpp_lifecycle::State &previous_state)
{

    /* Initialize all motors/remove all flags */
    controllers_init();
    pi3hat_->Cycle(pi3hat_input_);
    ::usleep(1000);

    /* Create rx_frame.id -> joint map */
    try
    {
        create_controller_joint_map();
    }
    catch(const std::exception& e)
    {
        RCLCPP_FATAL(*logger_, "%s", e.what());
        return hardware_interface::CallbackReturn::ERROR;
    }

    /* Get states with prepared controller -> joint map */
    controllers_get_states();

    RCLCPP_INFO(*logger_, "Hardware Interface successfully configured!");
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn Pi3HatHardwareInterface::on_activate(const rclcpp_lifecycle::State &previous_state)
{
    /* Set all commands, states and transmission passthrough to start state */
    for(int i = 0; i < joint_controller_number_; ++i)
    {
        controller_commands_[i].position_ = controller_start_positions_[i];
        controller_commands_[i].velocity_ = 0;
        controller_commands_[i].torque_ = 0;

        controller_states_[i].position_ = 0;
        controller_states_[i].velocity_ = 0;
        controller_states_[i].torque_ = 0;
        controller_states_[i].fault = 0;
        controller_states_[i].temperature_ = 0;

        controller_transmission_passthrough_[i].position_ = 0;
        controller_transmission_passthrough_[i].velocity_ = 0;
        controller_transmission_passthrough_[i].torque_ = 0;


        joint_commands_[i].position_ = 0;
        joint_commands_[i].velocity_ = 0;
        joint_commands_[i].torque_ = 0;

        joint_states_[i].position_ = 0;
        joint_states_[i].velocity_ = 0;
        joint_states_[i].torque_ = 0;
        joint_states_[i].fault = 0;
        joint_states_[i].temperature_ = 0;  

        joint_transmission_passthrough_[i].position_ = 0;
        joint_transmission_passthrough_[i].velocity_ = 0;
        joint_transmission_passthrough_[i].torque_ = 0;
    }

    
    /* Send start command to motors */
    RCLCPP_INFO(*logger_, "Motors reaching starting position!");
    double error_sum = NaN;

    while(error_sum > 0.01)
    {
        error_sum = 0;
        controllers_start_up();
        pi3hat_->Cycle(pi3hat_input_);
        ::usleep(5000);
        controllers_get_states();
        for(int i = 0; i < joint_controller_number_; ++i)
        {
            error_sum += std::abs(controller_states_[i].position_ - controller_commands_[i].position_);
        }
    }

    RCLCPP_INFO(*logger_, "Motors reached starting position!");
    RCLCPP_INFO(*logger_, "Hardware Interface successfully activated!!");
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn Pi3HatHardwareInterface::on_deactivate(const rclcpp_lifecycle::State &previous_state)
{
    /* Set all commands, states and transmission passthrough to end state */
    for(int i = 0; i < joint_controller_number_; ++i)
    {
        controller_commands_[i].position_ = controller_start_positions_[i];
        controller_commands_[i].velocity_ = 0;
        controller_commands_[i].torque_ = 0;

        controller_states_[i].position_ = 0;
        controller_states_[i].velocity_ = 0;
        controller_states_[i].torque_ = 0;
        controller_states_[i].fault = 0;
        controller_states_[i].temperature_ = 0;

        controller_transmission_passthrough_[i].position_ = 0;
        controller_transmission_passthrough_[i].velocity_ = 0;
        controller_transmission_passthrough_[i].torque_ = 0;


        joint_commands_[i].position_ = 0;
        joint_commands_[i].velocity_ = 0;
        joint_commands_[i].torque_ = 0;

        joint_states_[i].position_ = 0;
        joint_states_[i].velocity_ = 0;
        joint_states_[i].torque_ = 0;
        joint_states_[i].fault = 0;
        joint_states_[i].temperature_ = 0;  

        joint_transmission_passthrough_[i].position_ = 0;
        joint_transmission_passthrough_[i].velocity_ = 0;
        joint_transmission_passthrough_[i].torque_ = 0;
    }

    
    /* Send end commands to motors */
    RCLCPP_INFO(*logger_, "Motors reaching ending position!");
    double error_sum = NaN;
    
    while(error_sum > 0.01)
    {
        error_sum = 0;
        controllers_start_up();
        pi3hat_->Cycle(pi3hat_input_);
        ::usleep(5000);
        controllers_get_states();
        for(int i = 0; i < joint_controller_number_; ++i)
        {
            error_sum += std::abs(controller_states_[i].position_ - controller_commands_[i].position_);
        }
    }
    
    RCLCPP_INFO(*logger_, "Motors reached ending position!");
    RCLCPP_INFO(*logger_, "Hardware Interface successfully deactivated!");
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn Pi3HatHardwareInterface::on_cleanup(const rclcpp_lifecycle::State &previous_state)
{

    /* Deinitialize all motors/remove all flags */
    controllers_init();
    pi3hat_->Cycle(pi3hat_input_);
    ::usleep(1000);

    /* Get states with prepared controller -> joint map */
    controllers_get_states();

    RCLCPP_INFO(*logger_, "Hardware Interface successfully cleanuped!");
    return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::CommandInterface> Pi3HatHardwareInterface::export_command_interfaces()
{
    std::vector<hardware_interface::CommandInterface> command_interfaces;

    /* Joint commands (before joint -> controller transformation)*/
    for (int i = 0; i < joint_controller_number_; i++)
    {   
        if(!(info_.joints[i].command_interfaces.size() > 0))
        {
            RCLCPP_WARN(*logger_, "Zero command interfaces for joint %s!", info_.joints[i].name);
        }
        for(const auto& command_interface: info_.joints[i].command_interfaces)
        {
            if(command_interface.name == hardware_interface::HW_IF_POSITION)
            {
                command_interfaces.emplace_back(hardware_interface::CommandInterface(
                    info_.joints[i].name, hardware_interface::HW_IF_POSITION, &(joint_commands_[i].position_)));
            }
            else if(command_interface.name == hardware_interface::HW_IF_VELOCITY)
            {
                command_interfaces.emplace_back(hardware_interface::CommandInterface(
                    info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &(joint_commands_[i].velocity_)));
            }
            else if(command_interface.name == hardware_interface::HW_IF_EFFORT)
            {
                command_interfaces.emplace_back(hardware_interface::CommandInterface(
                    info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &(joint_commands_[i].torque_)));
            }
            else
            {
                RCLCPP_WARN(*logger_, "%s is wrong type of command interface, omitted!", command_interface.name);
            }
        }
    }

    return command_interfaces;
}

std::vector<hardware_interface::StateInterface> Pi3HatHardwareInterface::export_state_interfaces()
{
    std::vector<hardware_interface::StateInterface> state_interfaces;

    /* Joint states (after controller -> joint transformation)*/
    for (int i = 0; i < joint_controller_number_; i++)
    {
        if(!(info_.joints[i].state_interfaces.size() > 0))
        {
            RCLCPP_WARN(*logger_, "Zero state interfaces for joint %s!", info_.joints[i].name);
        }
        for(const auto& state_interface: info_.joints[i].state_interfaces)
        {
            if(state_interface.name == hardware_interface::HW_IF_POSITION)
            {
                state_interfaces.emplace_back(hardware_interface::StateInterface(
                    info_.joints[i].name, hardware_interface::HW_IF_POSITION, &(joint_states_[i].position_)));
            }
            if(state_interface.name == hardware_interface::HW_IF_VELOCITY)
            {
                state_interfaces.emplace_back(hardware_interface::StateInterface(
                    info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &(joint_states_[i].velocity_)));
            }
            if(state_interface.name == hardware_interface::HW_IF_EFFORT)
            {
                state_interfaces.emplace_back(hardware_interface::StateInterface(
                    info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &(joint_states_[i].torque_)));
            }
            if(state_interface.name == "temperature")
            {
                state_interfaces.emplace_back(hardware_interface::StateInterface(
                    info_.joints[i].name, "temperature", &(joint_states_[i].temperature_)));
            }
            else
            {
                RCLCPP_WARN(*logger_, "%s is wrong type of state interface, omitted!", state_interface.name);
            }
        }
    }

    /* IMU states (after IMUTransform transformation) */
    if(info_.sensors.size() == 0)
    {
        RCLCPP_WARN(*logger_, "IMU: state interface was not configured!");
        return state_interfaces;
    }

    if(info_.sensors[0].state_interfaces.size() != 10)
    {
        RCLCPP_WARN(*logger_, "IMU: some states were not configured!");
    }
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        "imu_sensor", "orientation.x", &attitude_.attitude.x));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        "imu_sensor", "orientation.y", &attitude_.attitude.y));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        "imu_sensor", "orientation.z", &attitude_.attitude.z));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        "imu_sensor", "orientation.w", &attitude_.attitude.w));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        "imu_sensor", "angular_velocity.x", &attitude_.rate_dps.x));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        "imu_sensor", "angular_velocity.y", &attitude_.rate_dps.y));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        "imu_sensor", "angular_velocity.z", &attitude_.rate_dps.z));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        "imu_sensor", "linear_acceleration.x", &attitude_.accel_mps2.x));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        "imu_sensor", "linear_acceleration.y", &attitude_.accel_mps2.y));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        "imu_sensor", "linear_acceleration.z", &attitude_.accel_mps2.z));

    return state_interfaces;
}


hardware_interface::return_type Pi3HatHardwareInterface::write(const rclcpp::Time &time, const rclcpp::Duration &period)
{
    for (int i = 0; i < joint_controller_number_; ++i)
    {
        if (std::isnan(joint_commands_[i].position_) || std::isnan(joint_commands_[i].velocity_) || std::isnan(joint_commands_[i].torque_))
        {
            RCLCPP_WARN(rclcpp::get_logger("Pi3HatHardwareInterface"), "NaN command for actuator");
            break;
        }
    }

    joint_to_controller_transform();

    controllers_make_commands();
    
    mjbots::pi3hat::Pi3Hat::Output result = pi3hat_->Cycle(pi3hat_input_);

    if (result.error)
    {
        RCLCPP_ERROR(rclcpp::get_logger("Pi3HatHardwareInterface"), "Pi3Hat::Cycle() failed!");
        return hardware_interface::return_type::ERROR;
    }

    if (result.attitude_present)
    {
        imu_transform_.transform_attitude(attitude_);
    }

    if(result.rx_can_size > 0)
    {
        controllers_get_states();
    }

    controller_to_joint_transform();
    
    return hardware_interface::return_type::OK;
}

hardware_interface::return_type Pi3HatHardwareInterface::read(const rclcpp::Time &time, const rclcpp::Duration &period)
{
    return hardware_interface::return_type::OK;
}

void Pi3HatHardwareInterface::joint_to_controller_transform()
{
    for(int i = 0; i < joint_controller_number_; ++i)
    {
        joint_transmission_passthrough_[i].position_ = joint_commands_[i].position_;
        joint_transmission_passthrough_[i].velocity_ = joint_commands_[i].velocity_;
        joint_transmission_passthrough_[i].torque_ = joint_commands_[i].torque_;
    }

    std::for_each(
    transmissions_.begin(), transmissions_.end(),
    [](auto & transmission) { transmission->joint_to_actuator(); });

    for(int i = 0; i < joint_controller_number_; ++i)
    {
        controller_commands_[i].position_ = controller_transmission_passthrough_[i].position_;
        controller_commands_[i].velocity_ = controller_transmission_passthrough_[i].velocity_;
        controller_commands_[i].torque_ = controller_transmission_passthrough_[i].torque_;
    }
}

void Pi3HatHardwareInterface::controller_to_joint_transform()
{
    for(int i = 0; i < joint_controller_number_; ++i)
    {
        controller_transmission_passthrough_[i].position_ = controller_states_[i].position_;
        controller_transmission_passthrough_[i].velocity_ = controller_states_[i].velocity_;
        controller_transmission_passthrough_[i].torque_ = controller_states_[i].torque_;
    }

    std::for_each(
    transmissions_.begin(), transmissions_.end(),
    [](auto & transmission) { transmission->actuator_to_joint(); });

    for(int i = 0; i < joint_controller_number_; ++i)
    {
        joint_states_[i].position_ = joint_transmission_passthrough_[i].position_;
        joint_states_[i].velocity_ = joint_transmission_passthrough_[i].velocity_;
        joint_states_[i].torque_ = joint_transmission_passthrough_[i].torque_;
    }
}

/* TRANSMISSION FUNCTIONS */
void Pi3HatHardwareInterface::append_joint_handles(std::vector<transmission_interface::JointHandle>& joint_handles, const std::string joint_name, const int joint_index)
{
    transmission_interface::JointHandle joint_handle_position(joint_name, hardware_interface::HW_IF_POSITION, 
     &joint_transmission_passthrough_[joint_index].position_);
    joint_handles.push_back(joint_handle_position);

    transmission_interface::JointHandle joint_handle_velocity(joint_name, hardware_interface::HW_IF_VELOCITY, 
     &joint_transmission_passthrough_[joint_index].velocity_);
    joint_handles.push_back(joint_handle_velocity);

    transmission_interface::JointHandle joint_handle_torque(joint_name, hardware_interface::HW_IF_EFFORT,
     &joint_transmission_passthrough_[joint_index].torque_);
    joint_handles.push_back(joint_handle_torque);
}

void Pi3HatHardwareInterface::append_actuator_handles(std::vector<transmission_interface::ActuatorHandle>& actuator_handles, const std::string actuator_name, const int actuator_index)
{
    transmission_interface::ActuatorHandle actuator_handle_position(actuator_name, hardware_interface::HW_IF_POSITION,
     &controller_transmission_passthrough_[actuator_index].position_);
    actuator_handles.push_back(actuator_handle_position);

    transmission_interface::ActuatorHandle actuator_handle_velocity(actuator_name, hardware_interface::HW_IF_VELOCITY, 
     &controller_transmission_passthrough_[actuator_index].velocity_);
    actuator_handles.push_back(actuator_handle_velocity);

    transmission_interface::ActuatorHandle actuator_handle_torque(actuator_name, hardware_interface::HW_IF_EFFORT,
     &controller_transmission_passthrough_[actuator_index].torque_);
    actuator_handles.push_back(actuator_handle_torque);
}


void Pi3HatHardwareInterface::load_transmission_data(const hardware_interface::TransmissionInfo& transmission_info, 
        transmission_interface::TransmissionSharedPtr& transmission, transmission_interface::TransmissionLoader& loader)
{
    try
    {
        transmission = loader.load(transmission_info);
    }
    catch (const transmission_interface::TransmissionInterfaceException & exc)
    {
        RCLCPP_FATAL(*logger_, "Error while loading %s: %s", transmission_info.name.c_str(), exc.what());
        return;
    }
}

void Pi3HatHardwareInterface::create_transmission_interface(const hardware_interface::HardwareInfo &info)
{
    if(info.transmissions.size() == 0) return;
    
    /* Prepare loaders */
    transmission_interface::SimpleTransmissionLoader simple_loader;
    transmission_interface::FourBarLinkageTransmissionLoader fbl_loader;
    transmission_interface::DifferentialTransmissionLoader diff_loader;

    /* Prepare joint names */
    std::vector<std::string> joint_names;
    for(const auto& joint: info.joints)
    {
        joint_names.push_back(joint.name);
    }

    /* For fast computation, transmissions will be sorted by type */

    /* Simple transmissions */
    for(const auto& transmission_info: info.transmissions)
    {
        if(transmission_info.type == "transmission_interface/SimpleTransmission")
        {
            try
            {
                create_simple_transmission(transmission_info, simple_loader, joint_names);
            }
            catch(const transmission_interface::TransmissionInterfaceException& e)
            {
                throw;
            }
            
        }
    }

    /* FourBarLinkage transmissions */
    for(const auto& transmission_info: info.transmissions)
    {
        if(transmission_info.type == "transmission_interface/FourBarLinkageTransmission")
        {
            try
            {
                create_fbl_transmission(transmission_info, fbl_loader, joint_names);
            }
            catch(const transmission_interface::TransmissionInterfaceException& e)
            {
                throw;
            }
            
        }
    }

    /* Differential transmissions */
    for(const auto& transmission_info: info.transmissions)
    {
        if(transmission_info.type == "transmission_interface/DifferentialTransmission")
        {
            try
            {
                create_diff_transmission(transmission_info, diff_loader, joint_names);
            }
            catch(const transmission_interface::TransmissionInterfaceException& e)
            {
                throw;
            }
        }
    }
}

void Pi3HatHardwareInterface::create_simple_transmission(const hardware_interface::TransmissionInfo& transmission_info,
        transmission_interface::SimpleTransmissionLoader& loader, const std::vector<std::string>& joint_names)
{
    RCLCPP_INFO(*logger_, "Simple transmission initialization starting!");

    std::shared_ptr<transmission_interface::Transmission> transmission;
    if (transmission_info.type != "transmission_interface/SimpleTransmission")
    {
        RCLCPP_FATAL(*logger_, "This is not SimpleTransmission!");
        throw transmission_interface::TransmissionInterfaceException("This is not SimpleTransmission!"); // this should not happen!
    }

    load_transmission_data(transmission_info, transmission, loader);

    if(transmission_info.joints.size() != 1)
    {
        RCLCPP_FATAL(*logger_, "Invalid number of joints in SimpleTransmission!");
        throw transmission_interface::TransmissionInterfaceException("Invalid number of joints in SimpleTransmission!"); // this should not happen!
    }

    if(transmission_info.actuators.size() != 1)
    {
        RCLCPP_FATAL(*logger_, "Invalid number of actuators in SimpleTransmission!");
        throw transmission_interface::TransmissionInterfaceException("Invalid number of actuators in SimpleTransmission!"); // this should not happen!
    }

    /* Find joint index for transmission handels by name*/
    std::vector<std::string>::const_iterator joint_it = std::find(joint_names.begin(), 
    joint_names.end(), transmission_info.joints[0].name);
    int joint_index = std::distance(joint_names.begin(), joint_it);

    /* Joint handels */
    std::vector<transmission_interface::JointHandle> joint_handles;
    append_joint_handles(joint_handles, transmission_info.joints[0].name, joint_index);
    
    /* Actuator handels */
    std::vector<transmission_interface::ActuatorHandle> actuator_handles;
    append_actuator_handles(actuator_handles, transmission_info.actuators[0].name, joint_index);

    try
    {
        transmission->configure(joint_handles, actuator_handles);
    }
    catch (const transmission_interface::TransmissionInterfaceException & exc)
    {
        RCLCPP_FATAL(*logger_, "Error while loading %s: %s", transmission_info.name.c_str(), exc.what());
        throw;
    }

    transmissions_.push_back(transmission);
    
    RCLCPP_INFO(*logger_, "Simple transmissions initialized!");
}

void Pi3HatHardwareInterface::create_fbl_transmission(const hardware_interface::TransmissionInfo& transmission_info, 
        transmission_interface::FourBarLinkageTransmissionLoader& loader, const std::vector<std::string>& joint_names)
{
    RCLCPP_INFO(*logger_, "FourBarLinkage transmissions initialization starting!");

    std::shared_ptr<transmission_interface::Transmission> transmission;
    if (transmission_info.type != "transmission_interface/FourBarLinkageTransmission")
    {
        RCLCPP_FATAL(*logger_, "This is not FourBarLinkageTransmission!");
        throw transmission_interface::TransmissionInterfaceException("This is not FourBarLinkageTransmission!"); // this should not happen!
    }

    load_transmission_data(transmission_info, transmission, loader);

    if(transmission_info.joints.size() != 2)
    {
        RCLCPP_FATAL(*logger_, "Invalid number of joints in FourBarLinkageTransmission!");
        throw transmission_interface::TransmissionInterfaceException("Invalid number of joints in FourBarLinkageTransmission!"); // this should not happen!
    }
    if(transmission_info.actuators.size() != 2)
    {
        RCLCPP_FATAL(*logger_, "Invalid number of actuators in FourBarLinkageTransmission!");
        throw transmission_interface::TransmissionInterfaceException("Invalid number of actuators in FourBarLinkageTransmission!"); // this should not happen!
    }

    /* Find joints indexes for transmission handels by name*/
    std::vector<std::string>::const_iterator joint_it_1 = std::find(joint_names.begin(), 
    joint_names.end(), transmission_info.joints[0].name);
    int joint_index_1 = std::distance(joint_names.begin(), joint_it_1);

    std::vector<std::string>::const_iterator joint_it_2 = std::find(joint_names.begin(), 
    joint_names.end(), transmission_info.joints[1].name);
    int joint_index_2 = std::distance(joint_names.begin(), joint_it_2);

    /* Joint handels */
    std::vector<transmission_interface::JointHandle> joint_handles;
    append_joint_handles(joint_handles, transmission_info.joints[0].name, joint_index_1);
    append_joint_handles(joint_handles, transmission_info.joints[1].name, joint_index_2);
    
    /* Actuator handels */
    std::vector<transmission_interface::ActuatorHandle> actuator_handles;
    append_actuator_handles(actuator_handles, transmission_info.actuators[0].name, joint_index_1);
    append_actuator_handles(actuator_handles, transmission_info.actuators[1].name, joint_index_2);

    try
    {
        transmission->configure(joint_handles, actuator_handles);
    }
    catch (const transmission_interface::TransmissionInterfaceException & exc)
    {
        RCLCPP_FATAL(*logger_, "Error while loading %s: %s", transmission_info.name.c_str(), exc.what());
        throw;
    }

    transmissions_.push_back(transmission);
    
    RCLCPP_INFO(*logger_, "FourBarLinkage transmissions initialized!");
}

void Pi3HatHardwareInterface::create_diff_transmission(const hardware_interface::TransmissionInfo& transmission_info, 
        transmission_interface::DifferentialTransmissionLoader& loader, const std::vector<std::string>& joint_names)
{
    RCLCPP_INFO(*logger_, "Differential transmissions initialization starting!");

    std::shared_ptr<transmission_interface::Transmission> transmission;
    if (transmission_info.type != "transmission_interface/DifferentialTransmission")
    {
        RCLCPP_FATAL(*logger_, "This is not DifferentialTransmission!");
        throw transmission_interface::TransmissionInterfaceException("This is not DifferentialTransmission!"); // this should not happen!
    }
    
    load_transmission_data(transmission_info, transmission, loader);

    if(transmission_info.joints.size() != 2)
    {
        RCLCPP_FATAL(*logger_, "Invalid number of joints in DifferentialTransmission!");
        throw transmission_interface::TransmissionInterfaceException("Invalid number of joints in DifferentialTransmission!");; // this should not happen!
    }
    if(transmission_info.actuators.size() != 2)
    {
        RCLCPP_FATAL(*logger_, "Invalid number of actuators in DifferentialTransmission!");
        throw transmission_interface::TransmissionInterfaceException("Invalid number of actuators in DifferentialTransmission!"); // this should not happen!
    }

    /* Find joints indexes for transmission handels by name*/
    std::vector<std::string>::const_iterator joint_it_1 = std::find(joint_names.begin(), 
    joint_names.end(), transmission_info.joints[0].name);
    int joint_index_1 = std::distance(joint_names.begin(), joint_it_1);

    std::vector<std::string>::const_iterator joint_it_2 = std::find(joint_names.begin(), 
    joint_names.end(), transmission_info.joints[1].name);
    int joint_index_2 = std::distance(joint_names.begin(), joint_it_2);

    /* Joint handels */
    std::vector<transmission_interface::JointHandle> joint_handles;
    append_joint_handles(joint_handles, transmission_info.joints[0].name, joint_index_1);
    append_joint_handles(joint_handles, transmission_info.joints[1].name, joint_index_2);
    
    /* Actuator handels */
    std::vector<transmission_interface::ActuatorHandle> actuator_handles;
    append_actuator_handles(actuator_handles, transmission_info.actuators[0].name, joint_index_1);
    append_actuator_handles(actuator_handles, transmission_info.actuators[1].name, joint_index_2);

    try
    {
        transmission->configure(joint_handles, actuator_handles);
    }
    catch (const transmission_interface::TransmissionInterfaceException & exc)
    {
        RCLCPP_FATAL(*logger_, "Error while loading %s: %s", transmission_info.name.c_str(), exc.what());
        throw;
    }

    transmissions_.push_back(transmission);
    
    RCLCPP_INFO(*logger_, "Differential transmissions initialized!");
}

void Pi3HatHardwareInterface::add_controller_bridge(const ControllerParameters& params, const WrapperType type)
{
    std::unique_ptr<ControllerWrapper> wrapper_ptr; 
    switch(type)
    {
        case Moteus:
            wrapper_ptr = make_moteus_wrapper(params);
            break;
        default:
            throw std::invalid_argument("Motor type doesn't exist!");
            break;
    }

    controller_interface::ControllerBridge bridge = controller_interface::ControllerBridge(std::move(wrapper_ptr), params);
    controller_bridges_.push_back(std::move(bridge));
}

Pi3HatHardwareInterface::WrapperType Pi3HatHardwareInterface::choose_wrapper_type(const std::string& type)
{
    if(type == "moteus")
    {
        return WrapperType::Moteus;
    }
    return WrapperType::Moteus;
}

ControllerParameters Pi3HatHardwareInterface::get_controller_parameters(const hardware_interface::ComponentInfo& joint_info)
{
    ControllerParameters params;
    try
    {
        params.bus_ = std::stoi(joint_info.parameters.at("controller_can_bus"));
        params.id_ = std::stoi(joint_info.parameters.at("controller_can_id"));
        params.direction_ = std::stoi(joint_info.parameters.at("motor_direction"));
        params.position_offset_ = std::stod(joint_info.parameters.at("motor_position_offset"));
        params.position_max_ = std::stod(joint_info.parameters.at("motor_position_max"));
        params.position_min_ = std::stod(joint_info.parameters.at("motor_position_min"));
        params.velocity_max_ = std::stod(joint_info.parameters.at("motor_velocity_max"));
        params.torque_max_ = std::stod(joint_info.parameters.at("motor_torque_max"));
    }
    catch(const std::exception& e)
    {
        throw;
    }

    return params;
}

void Pi3HatHardwareInterface::controllers_init()
{
    for(int i = 0; i < joint_controller_number_; ++i)
    {
        controller_bridges_[i].initialize(tx_can_frames_[i]);
    }
}

void Pi3HatHardwareInterface::controllers_start_up()
{
    for(int i = 0; i < joint_controller_number_; ++i)
    {
        controller_bridges_[i].start_up(tx_can_frames_[i],controller_commands_[i]);
    }
}

void Pi3HatHardwareInterface::controllers_make_commands()
{
    for(int i = 0; i < joint_controller_number_; ++i)
    {
        controller_bridges_[i].make_command(tx_can_frames_[i], controller_commands_[i]);
    }
}

void Pi3HatHardwareInterface::controllers_get_states()
{
    for(int i = 0; i < joint_controller_number_; ++i)
    {
        int joint_id = controller_joint_map_.at(rx_can_frames_[i].id);
        controller_bridges_[joint_id].get_state(rx_can_frames_[i], controller_states_[joint_id]);
    }
}

void Pi3HatHardwareInterface::create_controller_joint_map()
{
    for(int i = 0; i < joint_controller_number_; ++i)
    {
        int joint_id = i;
        int controller_id = tx_can_frames_[i].id;
        for(int j = 0; j < joint_controller_number_; ++j)
        {
            if(controller_id == rx_can_frames_[j].id)
            {
                std::pair<int, int> controller_joint_pair(controller_id, joint_id);
                controller_joint_map_.emplace(controller_joint_pair);
            }
        }
    }
    if(controller_joint_map_.size() != joint_controller_number_)
    {
        throw std::logic_error("Controller -> Joint map has diffrent length!");
    }
}

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  pi3hat_hardware_interface::Pi3HatHardwareInterface,
  hardware_interface::SystemInterface)