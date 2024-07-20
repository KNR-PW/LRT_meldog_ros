#include "../include/pi3hat_hardware_interface/pi3hat_hardware_interface.hpp"

using namespace pi3hat_hardware_interface;
using namespace actuator_wrappers;

hardware_interface::CallbackReturn Pi3HatHardwareInterface::on_init(const hardware_interface::HardwareInfo &info)
{
    if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS)
    {
        return hardware_interface::CallbackReturn::ERROR;
    }
    logger_ = std::make_unique<rclcpp::Logger>(
    rclcpp::get_logger("Pi3HatHardwareInterface"));

    hw_actuator_commands_.resize(info_.joints.size(), ActuatorState{std::numeric_limits<double>::quiet_NaN(),
     std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::quiet_NaN()});

    hw_actuator_states_.resize(info_.joints.size(), ActuatorState{std::numeric_limits<double>::quiet_NaN(),
     std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::quiet_NaN()});
    
    for (const hardware_interface::ComponentInfo &joint : info_.joints)
    {
        hw_actuator_can_buses_.push_back(std::stoi(joint.parameters.at("can_channel")));
        hw_actuator_can_ids_.push_back(std::stoi(joint.parameters.at("can_id")));  
        hw_actuator_position_mins_.push_back(std::stod(joint.parameters.at("position_min")));
        hw_actuator_position_maxs_.push_back(std::stod(joint.parameters.at("position_max")));
        hw_actuator_velocity_maxs_.push_back(std::stod(joint.parameters.at("velocity_max")));
        hw_actuator_effort_maxs_.push_back(std::stod(joint.parameters.at("effort_max")));
        hw_actuator_position_offsets_.push_back(std::stod(joint.parameters.at("position_offset")));
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

void Pi3HatHardwareInterface::append_joint_handels(std::vector<transmission_interface::JointHandle>& joint_handles, std::string joint_name, int joint_index)
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
void Pi3HatHardwareInterface::append_actuator_handels(std::vector<transmission_interface::ActuatorHandle>& actuator_handles, std::string actuator_name, int actuator_index)
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

    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn Pi3HatHardwareInterface::create_simple_transmissions(const hardware_interface::TransmissionInfo& transmission_info,
        transmission_interface::SimpleTransmissionLoader& loader, const std::vector<std::string>& joint_names)
{
    RCLCPP_INFO(*logger_, "Simple transmissions initialization starting!");
    for (const auto & transmission_info : info_.transmissions)
    {
        std::shared_ptr<transmission_interface::Transmission> transmission;

        if (transmission_info.type != "transmission_interface/SimpleTransmission")
        {
            continue;
        }

        load_transmission_data(transmission_info, transmission, loader);

        std::vector<transmission_interface::JointHandle> joint_handles;
        std::vector<transmission_interface::ActuatorHandle> actuator_handles;
    
        if(transmission_info.joints.size() != 1)
        {
            RCLCPP_FATAL(*logger_, "Invalid number of joints in SimpleTransmission!");
            return hardware_interface::CallbackReturn::ERROR;
        }

        if(transmission_info.actuators.size() != 1)
        {
            RCLCPP_FATAL(*logger_, "Invalid number of actuators in SimpleTransmission!");
            return hardware_interface::CallbackReturn::ERROR;
        }

        /* Find joint index for transmission handels by name*/
        std::vector<std::string>::const_iterator joint_it = std::find(joint_names.begin(), 
        joint_names.end(), transmission_info.joints[0].name);
        int joint_index = std::distance(joint_names.begin(), joint_it);

        /* Joint handels */
        std::vector<transmission_interface::JointHandle> joint_handels;
        append_joint_handels(joint_handels, transmission_info.joints[0].name, joint_index);
        
        /* Actuator handels */
        std::vector<transmission_interface::ActuatorHandle> actuator_handels;
        append_actuator_handels(actuator_handels, transmission_info.actuators[0].name, joint_index);

        transmission->configure(joint_handles, actuator_handles);
        transmissions_.push_back(transmission);
    }
    RCLCPP_INFO(*logger_, "Simple transmissions initialized!");

    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn Pi3HatHardwareInterface::create_fbl_transmissions(const hardware_interface::TransmissionInfo& transmission_info, 
        transmission_interface::FourBarLinkageTransmissionLoader& loader, const std::vector<std::string>& joint_names)
{
    RCLCPP_INFO(*logger_, "Simple transmissions initialization starting!");
    for (const auto & transmission_info : info_.transmissions)
    {
        std::shared_ptr<transmission_interface::Transmission> transmission;

        if (transmission_info.type != "transmission_interface/FourBarLinkageTransmission")
        {
            continue;
        }

        load_transmission_data(transmission_info, transmission, loader);

        std::vector<transmission_interface::JointHandle> joint_handles;
        std::vector<transmission_interface::ActuatorHandle> actuator_handles;

        if(transmission_info.joints.size() != 2)
        {
            RCLCPP_FATAL(*logger_, "Invalid number of joints in SimpleTransmission!");
            return hardware_interface::CallbackReturn::ERROR;
        }

        if(transmission_info.actuators.size() != 2)
        {
            RCLCPP_FATAL(*logger_, "Invalid number of actuators in SimpleTransmission!");
            return hardware_interface::CallbackReturn::ERROR;
        }


    }
    return hardware_interface::CallbackReturn::SUCCESS;
}