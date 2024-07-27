#include "../include/pi3hat_hardware_interface/pi3hat_hardware_interface.hpp"

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

    controller_commands_.resize(info_.joints.size());
    controller_states_.resize(info_.joints.size());
    controller_transmission_passthrough_.resize(info_.joints.size());


    joint_commands_.resize(info_.joints.size());
    joint_states_.resize(info_.joints.size());
    joint_transmission_passthrough_.resize(info_.joints.size());

    joint_controller_number_ = info_.joints.size();

    /* Prepare controller bridges */
    for (const hardware_interface::ComponentInfo &joint : info_.joints)
    {
        controller_interface::ControllerParameters params;
        WrapperType type;
        try
        {
            params = get_controller_parameters(joint);

            std::string type_string = joint.parameters.at("motor_type");
            type = choose_wrapper_type(type_string);
        }
        catch(const std::exception& e)
        {
            RCLCPP_FATAL(*logger_, "Error reading motor/controller parameters!");
            return hardware_interface::CallbackReturn::ERROR;
        }
        
        add_controller_bridge(params, type);
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
    pi3hat_input_.attitude = &attitude_;

    tx_can_frames_ = std::make_shared<mjbots::pi3hat::CanFrame[]>(new mjbots::pi3hat::CanFrame[info_.joints.size()]);
    rx_can_frames_ = std::make_shared<mjbots::pi3hat::CanFrame[]>(new mjbots::pi3hat::CanFrame[info_.joints.size()]);
    
    mjbots::pi3hat::Span<mjbots::pi3hat::CanFrame> rx_can_frames_span_(rx_can_frames_.get(), info_.joints.size()); 
    pi3hat_input_.rx_can = rx_can_frames_span_;
    mjbots::pi3hat::Span<mjbots::pi3hat::CanFrame> tx_can_frames_span_(tx_can_frames_.get(), info_.joints.size()); 
    pi3hat_input_.tx_can = tx_can_frames_span_;

    config.can[0] = can_config;
    config.can[1] = can_config;
    config.can[2] = can_config;
    config.can[3] = can_config;
    config.can[4] = can_config;

    /* Initialize the Pi3Hat and realtime options */
    pi3hat_ =  std::make_shared<mjbots::pi3hat::Pi3Hat>(config);
    mjbots::pi3hat::ConfigureRealtime(0);

    /* Initialize all motors/remove all flags */
    controllers_init();
    pi3hat_->Cycle(pi3hat_input_);

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
    
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn Pi3HatHardwareInterface::on_configure(const rclcpp_lifecycle::State &previous_state)
{
    /* Set all commands, states and transmission passthrough to 0 */
    for(int i = 0; i < controller_bridges.size(); ++i)
    {
        controller_commands_[i].position_ = 0;
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


        joint_commands_[i].position_;
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

    /* Start motors */
    controllers_start_up();
    pi3hat_->Cycle(pi3hat_input_);
    ::usleep(1000);
    controllers_get_states();
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
    std::vector<transmission_interface::JointHandle> joint_handles;
    std::vector<transmission_interface::ActuatorHandle> actuator_handles;

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
    std::vector<transmission_interface::JointHandle> joint_handles;
    std::vector<transmission_interface::ActuatorHandle> actuator_handles;

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
            wrapper_ptr = create_moteus_wrapper(params);
            break;
    }

    controller_bridges.push_back(controller_interface::ControllerBridge(std::move(wrapper_ptr), params));
}

std::unique_ptr<ControllerWrapper> Pi3HatHardwareInterface::create_moteus_wrapper(const ControllerParameters& params)
{
    /* moteus options */ 
    using mjbots::moteus::Controller;
    using controller_interface::MoteusWrapper;
    Controller::Options moteus_options;
    moteus_options.bus = params.bus_;
    moteus_options.id = params.id_;

    /* moteus command format (it will be copied to wrapper) */
    mjbots::moteus::PositionMode::Format format;
    format.feedforward_torque = mjbots::moteus::kFloat;
    format.maximum_torque = mjbots::moteus::kFloat;
    format.velocity_limit= mjbots::moteus::kFloat;
    moteus_options.position_format = format;

    /* moteus command (it will be copied to wrapper) */
    mjbots::moteus::PositionMode::Command moteus_command;
    moteus_command.maximum_torque = params.torque_max_;
    moteus_command.velocity_limit = params.velocity_max_;
    
    return std::make_unique<MoteusWrapper>(moteus_options, moteus_command);
}

Pi3HatHardwareInterface::WrapperType Pi3HatHardwareInterface::choose_wrapper_type(const std::string& type)
{
    if(type == "moteus")
    {
        return WrapperType::Moteus;
    };
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
    int size = controller_bridges.size();
    for(int i = 0; i < joint_controller_number_; ++i)
    {
        controller_bridges[i].initialize(tx_can_frames_[i]);
    }
}

void Pi3HatHardwareInterface::controllers_start_up()
{
    int size = controller_bridges.size();
    for(int i = 0; i < joint_controller_number_; ++i)
    {
        controller_bridges[i].start_up(tx_can_frames_[i],controller_commands_[i]);
    }
}

void Pi3HatHardwareInterface::controllers_make_commands()
{
    int size = controller_bridges.size();
    for(int i = 0; i < joint_controller_number_; ++i)
    {
        controller_bridges[i].make_command(tx_can_frames_[i], controller_commands_[i]);
    }
}

void Pi3HatHardwareInterface::controllers_get_states()
{
    int size = controller_bridges.size();
    for(int i = 0; i < joint_controller_number_; ++i)
    {
        int joint_id = controller_joint_map_.at(rx_can_frames_[i].id);
        controller_bridges[joint_id].get_state(rx_can_frames_[i], controller_states_[joint_id]);
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