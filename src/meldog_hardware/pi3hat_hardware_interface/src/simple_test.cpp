#include "../include/pi3hat/pi3hat.h"
#include "../include/pi3hat/realtime.h"
#include "../include/moteus/moteus.h"


static double GetNow() 
{
  struct timespec ts = {};
  ::clock_gettime(CLOCK_MONOTONIC_RAW, &ts);
  return static_cast<double>(ts.tv_sec) +
      static_cast<double>(ts.tv_nsec) / 1e9;
};

int main(int argc, char** argv)
{
    // moteus part
    using mjbots::moteus::Controller;
    Controller::Options moteus_options;
    moteus_options.bus = 1;
    moteus_options.id = 1;
    Controller moteus_controller(moteus_options);

    mjbots::moteus::PositionMode::Command moteus_command;

    //pi3hat part

    mjbots::pi3hat::Pi3Hat::Configuration pi3hat_configuration;
    pi3hat_configuration.attitude_rate_hz = 1000;

    mjbots::pi3hat::CanFrame tx_frame;
    mjbots::pi3hat::CanFrame rx_frame[10];
    tx_frame.id = 1; 
    tx_frame.bus = 1;
    tx_frame.expect_reply = true;
    mjbots::pi3hat::Span<mjbots::pi3hat::CanFrame> tx_span(&tx_frame, 1);
    mjbots::pi3hat::Span<mjbots::pi3hat::CanFrame> rx_span(rx_frame, 10);
    mjbots::pi3hat::Attitude attitude;

    mjbots::pi3hat::Pi3Hat::Input input;
    mjbots::pi3hat::Pi3Hat::Output pi3hat_output;
    input.tx_can = tx_span;
    input.rx_can = rx_span;
    input.attitude = &attitude;
    input.request_attitude = true;

    std::unique_ptr<mjbots::pi3hat::Pi3Hat> pi3hat = std::make_unique<mjbots::pi3hat::Pi3Hat>(pi3hat_configuration);

    mjbots::moteus::CanFdFrame can_fd_frame = moteus_controller.MakeStop();
    tx_frame.size = can_fd_frame.size;
    std::memcpy(tx_frame.data, can_fd_frame.data, can_fd_frame.size);

    pi3hat_output = pi3hat->Cycle(input);
    std::cout << "Started: " << pi3hat_output.rx_can_size << std::endl;
    // Buffer for printing info
    char buf[4096] = {};

    auto prev = GetNow();
    double frequency;
    std::string status_line;
    while(true)
    {
        auto now = GetNow();
        double position_ = 5 * sin(now - prev);
        moteus_command.position = position_;
        mjbots::moteus::CanFdFrame can_fd_frame = moteus_controller.MakePosition(moteus_command);

        /* Copy data from CANFD frame to CAN frame*/
        tx_frame.bus = can_fd_frame.bus;
        tx_frame.id = can_fd_frame.arbitration_id; // DZIEKI TEMU DZIALAAA
        tx_frame.size = can_fd_frame.size;
        std::memcpy(tx_frame.data, can_fd_frame.data, can_fd_frame.size);


        auto mesaure_time = GetNow() - now;
        frequency = 1/mesaure_time;
        pi3hat_output = pi3hat->Cycle(input);
        ::usleep(1000);
        mjbots::moteus::Query::Result result = mjbots::moteus::Query::Parse(rx_frame[0].data, rx_frame[0].size);
        double state_position = result.position;
        std::cout << "Attitude?: " << pi3hat_output.attitude_present << std::endl;
        std::cout << "Data size?: " << pi3hat_output.rx_can_size << std::endl;

    };
    return 0;
}
