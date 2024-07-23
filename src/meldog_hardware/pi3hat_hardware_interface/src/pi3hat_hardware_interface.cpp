#include "../include/pi3hat_hardware_interface/pi3hat_hardware_interface.hpp"

using namespace pi3hat_hardware_interface;
using namespace actuator_wrappers;

/* MAIN FUNCTIONS */

hardware_interface::CallbackReturn Pi3HatHardwareInterface::on_init(const hardware_interface::HardwareInfo &info)
{
    if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS)
    {
        return hardware_interface::CallbackReturn::ERROR;
    }

    logger_ = std::make_unique<rclcpp::Logger>(
    rclcpp::get_logger("Pi3HatHardwareInterface"));

    hw_actuator_commands_.resize(info_.joints.size());
    hw_actuator_states_.resize(info_.joints.size());
    hw_actuator_transmission_passthrough_.resize(info_.joints.size());


    hw_joint_commands_.resize(info_.joints.size());
    hw_joint_states_.resize(info_.joints.size());
    hw_joint_transmission_passthrough_.resize(info_.joints.size());
    
    for (const hardware_interface::ComponentInfo &joint : info_.joints)
    {
        actuator_wrappers::ActuatorParameters params;
        params.bus_ = std::stoi(joint.parameters.at("can_bus"));
        params.id_ = std::stoi(joint.parameters.at("can_id"));
        params.direction_ = std::stoi(joint.parameters.at("direction"));
        params.position_offset_ = std::stod(joint.parameters.at("position_offset"));
        std::string type_string = joint.parameters.at("motor_type");

        WrapperType type = choose_actuator_wrapper(type_string);

        for(const auto& command_interface: joint.command_interfaces)
        {
            if(command_interface.name == hardware_interface::HW_IF_POSITION)
            {
                params.position_max_ = std::stod(command_interface.max);
                params.position_min_ = std::stod(command_interface.min);

            }
            else if(command_interface.name == hardware_interface::HW_IF_VELOCITY)
            {
                params.velocity_max_ = std::stod(command_interface.max);
            }
            else if(command_interface.name == hardware_interface::HW_IF_EFFORT)
            {
                params.torque_max_ = std::stod(command_interface.max);
            }
        }
        add_actuator_wrapper(params, type);

    }
    
    /* Standard CAN config */
    mjbots::pi3hat::Pi3Hat::CanConfiguration can_config;

    /* Configure the Pi3Hat for 1000hz IMU sampling */ 
    mjbots::pi3hat::Pi3Hat::Configuration config;
    config.attitude_rate_hz = 1000;
    /* Set the mounting orientation of the IMU */
    config.mounting_deg.yaw = std::stod(info_.hardware_parameters.at("imu_mounting_deg.yaw"));
    config.mounting_deg.pitch = std::stod(info_.hardware_parameters.at("imu_mounting_deg.pitch"));
    config.mounting_deg.roll = std::stod(info_.hardware_parameters.at("imu_mounting_deg.roll"));
    /* Initialize the Pi3Hat input */ 
    pi3hat_input_ = mjbots::pi3hat::Pi3Hat::Input();
    pi3hat_input_.attitude = &attitude_;

    tx_can_frames_ = std::make_shared<mjbots::pi3hat::CanFrame[]>(new mjbots::pi3hat::CanFrame[info_.joints.size()]);
    rx_can_frames_ = std::make_shared<mjbots::pi3hat::CanFrame[]>(new mjbots::pi3hat::CanFrame[info_.joints.size()]);
    
    mjbots::pi3hat::Span<mjbots::pi3hat::CanFrame> rx_can_frames_span_(rx_can_frames_.get(), info_.joints.size()); 
    pi3hat_input_.rx_can = rx_can_frames_span_;
    mjbots::pi3hat::Span<mjbots::pi3hat::CanFrame> tx_can_frames_span_(tx_can_frames_.get(), info_.joints.size()); 
    pi3hat_input_.tx_can = tx_can_frames_span_;

    // Set up the CAN configuration
    for (size_t i = 0; i < info_.joints.size(); i++) // TUTAJ POMYSL O CO CMN
    {
        config.can[hw_actuator_can_buses_[i] - 1] = can_config;
        pi3hat_input_.tx_can[i].id = hw_actuator_can_ids_[i];
        pi3hat_input_.tx_can[i].bus = hw_actuator_can_buses_[i];
        pi3hat_input_.tx_can[i].expect_reply = true;
    }
    // Initialize the Pi3Hat
    pi3hat_ =  std::make_shared<mjbots::pi3hat::Pi3Hat>(config);

    /* Create actuator wrappers*/
    for(size_t i = 0; i < info_.joints.size(); i++)
    {
        auto options = mjbots::moteus::Controller::Options();
        options.id = hw_actuator_can_ids_[i];
        options.bus = hw_actuator_can_buses_[i];
        auto moteus_wrapper = MoteusWrapper(options, tx_can_frames_[i], rx_can_frames_[i],
        hw_actuator_commands_[i], hw_actuator_states_[i], mjbots::moteus::PositionMode::Command());
        moteus_wrappers.push_back(moteus_wrapper);
    }
}






/* TRANSMISSION FUNCTIONS */
void Pi3HatHardwareInterface::append_joint_handles(std::vector<transmission_interface::JointHandle>& joint_handles, const std::string joint_name, const int joint_index)
{
    transmission_interface::JointHandle joint_handle_position(joint_name, hardware_interface::HW_IF_POSITION, 
     &hw_joint_transmission_passthrough_[joint_index].position_);
    joint_handles.push_back(joint_handle_position);

    transmission_interface::JointHandle joint_handle_velocity(joint_name, hardware_interface::HW_IF_VELOCITY, 
     &hw_joint_transmission_passthrough_[joint_index].velocity_);
    joint_handles.push_back(joint_handle_velocity);

    transmission_interface::JointHandle joint_handle_torque(joint_name, hardware_interface::HW_IF_EFFORT,
     &hw_joint_transmission_passthrough_[joint_index].torque_);
    joint_handles.push_back(joint_handle_torque);
}

void Pi3HatHardwareInterface::append_actuator_handles(std::vector<transmission_interface::ActuatorHandle>& actuator_handles, const std::string actuator_name, const int actuator_index)
{
    transmission_interface::ActuatorHandle actuator_handle_position(actuator_name, hardware_interface::HW_IF_POSITION,
     &hw_actuator_transmission_passthrough_[actuator_index].position_);
    actuator_handles.push_back(actuator_handle_position);

    transmission_interface::ActuatorHandle actuator_handle_velocity(actuator_name, hardware_interface::HW_IF_VELOCITY, 
     &hw_actuator_transmission_passthrough_[actuator_index].velocity_);
    actuator_handles.push_back(actuator_handle_velocity);

    transmission_interface::ActuatorHandle actuator_handle_torque(actuator_name, hardware_interface::HW_IF_EFFORT,
     &hw_actuator_transmission_passthrough_[actuator_index].torque_);
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

hardware_interface::CallbackReturn Pi3HatHardwareInterface::create_transmission_interface(const hardware_interface::HardwareInfo &info)
{
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
            if(create_simple_transmission(transmission_info, simple_loader, joint_names) == hardware_interface::CallbackReturn::ERROR)
            {
                return hardware_interface::CallbackReturn::ERROR;
            }
        }
    }

    /* FourBarLinkage transmissions */
    for(const auto& transmission_info: info.transmissions)
    {
        if(transmission_info.type == "transmission_interface/FourBarLinkageTransmission")
        {
            if(create_fbl_transmission(transmission_info, fbl_loader, joint_names) == hardware_interface::CallbackReturn::ERROR)
            {
                return hardware_interface::CallbackReturn::ERROR;;
            }
        }
    }

    /* Differential transmissions */
    for(const auto& transmission_info: info.transmissions)
    {
        if(transmission_info.type == "transmission_interface/DifferentialTransmission")
        {
            if(create_diff_transmission(transmission_info, diff_loader, joint_names) == hardware_interface::CallbackReturn::ERROR)
            {
                return hardware_interface::CallbackReturn::ERROR;
            }
        }
    }
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn Pi3HatHardwareInterface::create_simple_transmission(const hardware_interface::TransmissionInfo& transmission_info,
        transmission_interface::SimpleTransmissionLoader& loader, const std::vector<std::string>& joint_names)
{
    RCLCPP_INFO(*logger_, "Simple transmission initialization starting!");

    std::shared_ptr<transmission_interface::Transmission> transmission;
    if (transmission_info.type != "transmission_interface/SimpleTransmission")
    {
        RCLCPP_FATAL(*logger_, "This is not SimpleTransmission!");
        return hardware_interface::CallbackReturn::ERROR; // this should not happen!
    }

    load_transmission_data(transmission_info, transmission, loader);

    if(transmission_info.joints.size() != 1)
    {
        RCLCPP_FATAL(*logger_, "Invalid number of joints in SimpleTransmission!");
        return hardware_interface::CallbackReturn::ERROR; // this should not happen!
    }

    if(transmission_info.actuators.size() != 1)
    {
        RCLCPP_FATAL(*logger_, "Invalid number of actuators in SimpleTransmission!");
        return hardware_interface::CallbackReturn::ERROR; // this should not happen!
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
        return hardware_interface::CallbackReturn::ERROR;
    }

    transmissions_.push_back(transmission);
    
    RCLCPP_INFO(*logger_, "Simple transmissions initialized!");

    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn Pi3HatHardwareInterface::create_fbl_transmission(const hardware_interface::TransmissionInfo& transmission_info, 
        transmission_interface::FourBarLinkageTransmissionLoader& loader, const std::vector<std::string>& joint_names)
{
    RCLCPP_INFO(*logger_, "FourBarLinkage transmissions initialization starting!");

    std::shared_ptr<transmission_interface::Transmission> transmission;
    if (transmission_info.type != "transmission_interface/FourBarLinkageTransmission")
    {
        RCLCPP_FATAL(*logger_, "This is not FourBarLinkageTransmission!");
        return hardware_interface::CallbackReturn::ERROR; // this should not happen!
    }

    load_transmission_data(transmission_info, transmission, loader);
    std::vector<transmission_interface::JointHandle> joint_handles;
    std::vector<transmission_interface::ActuatorHandle> actuator_handles;

    if(transmission_info.joints.size() != 2)
    {
        RCLCPP_FATAL(*logger_, "Invalid number of joints in FourBarLinkageTransmission!");
        return hardware_interface::CallbackReturn::ERROR; // this should not happen!
    }
    if(transmission_info.actuators.size() != 2)
    {
        RCLCPP_FATAL(*logger_, "Invalid number of actuators in FourBarLinkageTransmission!");
        return hardware_interface::CallbackReturn::ERROR; // this should not happen!
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
        return hardware_interface::CallbackReturn::ERROR;
    }

    transmissions_.push_back(transmission);
    
    RCLCPP_INFO(*logger_, "FourBarLinkage transmissions initialized!");

    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn Pi3HatHardwareInterface::create_diff_transmission(const hardware_interface::TransmissionInfo& transmission_info, 
        transmission_interface::DifferentialTransmissionLoader& loader, const std::vector<std::string>& joint_names)
{
    RCLCPP_INFO(*logger_, "Differential transmissions initialization starting!");

    std::shared_ptr<transmission_interface::Transmission> transmission;
    if (transmission_info.type != "transmission_interface/DifferentialTransmission")
    {
        RCLCPP_FATAL(*logger_, "This is not DifferentialTransmission!");
        return hardware_interface::CallbackReturn::ERROR; // this should not happen!
    }
    
    load_transmission_data(transmission_info, transmission, loader);
    std::vector<transmission_interface::JointHandle> joint_handles;
    std::vector<transmission_interface::ActuatorHandle> actuator_handles;

    if(transmission_info.joints.size() != 2)
    {
        RCLCPP_FATAL(*logger_, "Invalid number of joints in DifferentialTransmission!");
        return hardware_interface::CallbackReturn::ERROR; // this should not happen!
    }
    if(transmission_info.actuators.size() != 2)
    {
        RCLCPP_FATAL(*logger_, "Invalid number of actuators in DifferentialTransmission!");
        return hardware_interface::CallbackReturn::ERROR; // this should not happen!
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
        return hardware_interface::CallbackReturn::ERROR;
    }

    transmissions_.push_back(transmission);
    
    RCLCPP_INFO(*logger_, "Differential transmissions initialized!");

    return hardware_interface::CallbackReturn::SUCCESS;
}

void Pi3HatHardwareInterface::add_actuator_wrapper(const ActuatorParameters& params, const WrapperType type)
{
    switch(type)
    {
        case Moteus:
            /* moteus options */ 
            using mjbots::moteus::Controller;
            Controller::Options moteus_options;
            moteus_options.bus = params.bus_;
            moteus_options.id = params.id_;

            // moteus command format (it will be copied to wrapper)
            mjbots::moteus::PositionMode::Format format;
            format.feedforward_torque = mjbots::moteus::kFloat;
            format.maximum_torque = mjbots::moteus::kFloat;
            format.velocity_limit= mjbots::moteus::kFloat;
            moteus_options.position_format = format;

            //moteus command (it will be copied to wrapper)
            mjbots::moteus::PositionMode::Command moteus_command;

            moteus_wrappers.push_back(MoteusWrapper(params, moteus_options, moteus_command));
        break;
    }   
}

Pi3HatHardwareInterface::WrapperType Pi3HatHardwareInterface::choose_actuator_wrapper(const std::string& type)
{
    if(type == "moteus")
    {
        return WrapperType::Moteus;
    };
}