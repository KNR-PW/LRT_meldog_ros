#include "../../../include/actuator_wrappers/MoteusWrapper.hpp"



#include "../../../include/pi3hat/pi3hat.h"
#include "../../../include/pi3hat/realtime.h"
#include <iostream>
#include <cmath>
#include <unistd.h>
#include <string>



static double GetNow() 
{
  struct timespec ts = {};
  ::clock_gettime(CLOCK_MONOTONIC_RAW, &ts);
  return static_cast<double>(ts.tv_sec) +
      static_cast<double>(ts.tv_nsec) / 1e9;
};

template <class Derived>
void init(actuator_wrappers::ActuatorWrapperBase<Derived>& wrapper, mjbots::pi3hat::CanFrame& tx_frame)
{
    wrapper.init(tx_frame);
};

template <class Derived>
void command(actuator_wrappers::ActuatorWrapperBase<Derived>& wrapper, mjbots::pi3hat::CanFrame& tx_frame, actuator_wrappers::ActuatorCommand& command)
{
    wrapper.command_to_tx_frame(tx_frame, command);
};
template <class Derived>
void state(actuator_wrappers::ActuatorWrapperBase<Derived>& wrapper,const mjbots::pi3hat::CanFrame& rx_frame, actuator_wrappers::ActuatorState& state)
{
    wrapper.rx_frame_to_state(rx_frame, state);
}


int main(int argc, char** argv)
{
    // moteusues options
    using mjbots::moteus::Controller;
    Controller::Options moteus_1_options;
    moteus_1_options.bus = 1;
    moteus_1_options.id = 1;

    Controller::Options moteus_2_options;
    moteus_2_options.bus = 2;
    moteus_2_options.id = 2;

    // moteus command format (it will be copied to wrapper)
    mjbots::moteus::PositionMode::Format format;
    format.feedforward_torque = mjbots::moteus::kFloat;
    format.maximum_torque = mjbots::moteus::kFloat;
    moteus_1_options.position_format = format;
    moteus_2_options.position_format = format;

    //moteus command (it will be copied to wrapper)
    mjbots::moteus::PositionMode::Command moteus_command;


    // pi3hat 
    mjbots::pi3hat::Pi3Hat::Configuration pi3hat_configuration;
    pi3hat_configuration.attitude_rate_hz = 1000;


    std::vector<mjbots::pi3hat::CanFrame> tx_frame;
    std::vector<mjbots::pi3hat::CanFrame> rx_frame;
    tx_frame[0].expect_reply = true;
    tx_frame[1].expect_reply = true;

    mjbots::pi3hat::Span<mjbots::pi3hat::CanFrame> tx_span(tx_frame.data(), 2);
    mjbots::pi3hat::Span<mjbots::pi3hat::CanFrame> rx_span(rx_frame.data(), 2);
    mjbots::pi3hat::Attitude attitude;

    mjbots::pi3hat::Pi3Hat::Input input;

    input.tx_can = tx_span;
    input.rx_can = rx_span;
    input.attitude = &attitude;
    input.request_attitude = true;

    mjbots::pi3hat::Pi3Hat pi3hat(pi3hat_configuration);
    
    // pi3hat output
    mjbots::pi3hat::Pi3Hat::Output pi3hat_output;

    // moteus wrapper
    actuator_wrappers::ActuatorParameters params_1;
    params_1.direction_ = 1;
    params_1.position_max_ = 30;
    params_1.position_min_ = -30;
    params_1.velocity_max_ = 10;
    params_1.torque_max_ = 1;
    params_1.bus_ = 1;
    params_1.id_ = 1;

    actuator_wrappers::ActuatorParameters params_2;
    params_2.direction_ = 1;
    params_2.position_max_ = 10;
    params_2.position_min_ = -10;
    params_2.velocity_max_ = 10;
    params_2.torque_max_ = 1;
    params_2.bus_ = 2;
    params_2.id_ = 2;

    std::vector<actuator_wrappers::MoteusWrapper> actuator_wrappers;
    std::vector<actuator_wrappers::ActuatorCommand> actuator_commands;
    std::vector<actuator_wrappers::ActuatorState> actuator_states;

    actuator_wrappers.push_back(actuator_wrappers::MoteusWrapper(params_1, moteus_1_options, moteus_command));
    actuator_wrappers.push_back(actuator_wrappers::MoteusWrapper(params_2, moteus_2_options, moteus_command));
    
    actuator_commands.push_back(actuator_wrappers::ActuatorCommand());
    actuator_commands.push_back(actuator_wrappers::ActuatorCommand());
    
    actuator_states.push_back(actuator_wrappers::ActuatorState());
    actuator_states.push_back(actuator_wrappers::ActuatorState());


    std::cout << "Options for controllers succesfully initialized!" << std::endl;


    mjbots::pi3hat::ConfigureRealtime(0);
    std::cout << "Realtime control activated!" << std::endl;

    // set stop to moteus
    for(int i = 0; i < actuator_wrappers.size(); i++)
    {
        init(actuator_wrappers[i],tx_frame[i]);
    }
    pi3hat_output = pi3hat.Cycle(input);
    std::cout << "Controllers successfully started!" << std::endl;

    auto prev = GetNow();
    int frequency;
    while(true)
    {   
        auto now = GetNow();
        actuator_commands[0].position_ = 20 * cos(now - prev);
        actuator_commands[1].position_ = 10 * sin(now - prev);
        for(int i = 0; i < actuator_wrappers.size(); i++)
        {
            command(actuator_wrappers[i],tx_frame[i],actuator_commands[i]);
        }
        pi3hat_output = pi3hat.Cycle(input);
        ::usleep(1000);
        auto mesaure_time = GetNow() - now;
        frequency = (int) 1/mesaure_time;
        for(int i = 0; i < actuator_wrappers.size(); i++)
        {
            state(actuator_wrappers[i],rx_frame[i],actuator_states[i]);
        }
        ::printf("f = %d\n pos_1_command = %7.3f, pos_1_state = %7.3f)\n pos_2_command = %7.3f, pos_2_state = %7.3f)\r",
        frequency, actuator_commands[0].position_, actuator_states[0].position_, actuator_commands[1].position_, actuator_states[1].position_);
        ::fflush(::stdout);
    }

    return 0;
}