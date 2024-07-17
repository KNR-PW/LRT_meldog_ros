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


hardware_interface::CallbackReturn Pi3HatHardwareInterface::create_simple_transmissions(const hardware_interface::TransmissionInfo& transmission_info,
        transmission_interface::SimpleTransmissionLoader& loader)
{
    for (const auto & transmission_info : info_.transmissions)
    {
        if (transmission_info.type != "transmission_interface/SimpleTransmission")
        {
            continue;
        }
        std::shared_ptr<transmission_interface::Transmission> transmission;
        try
        {
            transmission = loader.load(transmission_info);
        }
        catch (const transmission_interface::TransmissionInterfaceException & exc)
        {
            RCLCPP_FATAL(*logger_, "Error while loading %s: %s", transmission_info.name.c_str(), exc.what());
            return hardware_interface::CallbackReturn::ERROR;
        }
        std::vector<transmission_interface::JointHandle> joint_handles;
        std::vector<transmission_interface::ActuatorHandle> actuator_handles;
    
        if(transmission_info.joints.size() != 1)
        {
            RCLCPP_FATAL(*logger_, "Invalid number of joints in SimpleTransmission!");
            return hardware_interface::CallbackReturn::ERROR;
        }
        transmission_interface::JointHandle joint_handle(transmission_info.joints[0].name,
        hardware_interface::HW_IF_POSITION, ); //pomysl jak tu dodac position odpowiedniego jointa!
        joint_handles.push_back()
    }
}