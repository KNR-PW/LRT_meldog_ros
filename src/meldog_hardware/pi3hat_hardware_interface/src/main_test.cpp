#include "../include/actuator_wrappers/MoteusWrapper.hpp"



#include "../include/pi3hat/pi3hat.h"
#include "../include/pi3hat/realtime.h"
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

int main(int argc, char** argv)
{
    // moteus options
    using mjbots::moteus::Controller;
    Controller::Options moteus_options;
    moteus_options.bus = 1;
    moteus_options.id = 1;

    // moteus command format (it will be copied to wrapper)
    mjbots::moteus::PositionMode::Format format;
    format.feedforward_torque = mjbots::moteus::kFloat;
    format.maximum_torque = mjbots::moteus::kFloat;
    moteus_options.position_format = format;

    //moteus command (it will be copied to wrapper)
    mjbots::moteus::PositionMode::Command moteus_command;


    // pi3hat 
    mjbots::pi3hat::Pi3Hat::Configuration pi3hat_configuration;
    pi3hat_configuration.attitude_rate_hz = 1000;

    mjbots::pi3hat::CanFrame tx_frame;
    mjbots::pi3hat::CanFrame rx_frame;
    tx_frame.id = 1; // TO DODAJ DO WRAPPERA!!
    tx_frame.bus = 1;
    tx_frame.expect_reply = true;
    mjbots::pi3hat::Span<mjbots::pi3hat::CanFrame> tx_span(&tx_frame, 1);
    mjbots::pi3hat::Span<mjbots::pi3hat::CanFrame> rx_span(&rx_frame, 1);
    mjbots::pi3hat::Attitude attitude;

    mjbots::pi3hat::Pi3Hat::Input input;

    input.tx_can = tx_span;
    input.rx_can = rx_span;
    input.attitude = &attitude;
    input.request_attitude = true;

    std::cout << "Options for controller succesfully initialized!" << std::endl;
    mjbots::pi3hat::Pi3Hat pi3hat(pi3hat_configuration);
    
    // pi3hat output
    mjbots::pi3hat::Pi3Hat::Output pi3hat_output;

    // moteus wrapper
    actuator_wrappers::ActuatorParameters params;
    params.direction_ = 1;
    params.position_max_ = 10;
    params.position_min_ = -10;
    params.velocity_max_ = 4;
    params.torque_max_ = 1;

    actuator_wrappers::MoteusWrapper moteus_wrapper(params, moteus_options, moteus_command);
    actuator_wrappers::ActuatorCommand actuator_command;
    actuator_command.velocity_ = 0;
    actuator_command.torque_ = 0;

    actuator_wrappers::ActuatorState actuator_state;

    std::cout << "Options for controller succesfully initialized!" << std::endl;


    mjbots::pi3hat::ConfigureRealtime(0);
    std::cout << "Realtime control activated!" << std::endl;

    // set stop to moteus
    moteus_wrapper.init(tx_frame);
    pi3hat_output = pi3hat.Cycle(input);
    std::cout << "Controller successfully started!" << std::endl;

    // Buffer for printing info
    char buf[4096] = {};

    auto prev = GetNow();
    double frequency;
    std::string status_line;
    while(true)
    {   
        auto now = GetNow();
        actuator_command.position_ = 10 * sin(now - prev);
        moteus_wrapper.command_to_tx_frame(tx_frame, actuator_command);
        pi3hat_output = pi3hat.Cycle(input);
        auto mesaure_time = GetNow() - now;
        frequency = 1/mesaure_time;
        moteus_wrapper.rx_frame_to_state(rx_frame, actuator_state);
        ::snprintf(buf, sizeof(buf) -1, "f/p/v/t=(%7.3f, %7.3f, %7.3f, %7.3f) ",
        frequency, actuator_state.position_, actuator_state.velocity_, actuator_state.torque_);
        status_line += buf;
        ::printf("%s\n  \r", status_line.c_str());
        ::fflush(::stdout);
    }

    return 0;
}