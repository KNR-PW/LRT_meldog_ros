#include <rclcpp/rclcpp.hpp>
#include <string>
#include <joint_controller_msgs/msg/joint_command.hpp>


int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  const std::string nodeName = "sinus_joint_test";
  const double amplitude = 4 * M_PI;
  const double frequency = 0.25; //[s]

  const double angularVelocity = 2 * M_PI * frequency;
  rclcpp::Node::SharedPtr sinusoidalNode = std::make_shared<rclcpp::Node>(nodeName);
  std::string topicName = "/joint_controller/joint_commands";
  const auto sinusPublisher = sinusoidalNode->create_publisher<joint_controller_msgs::msg::JointCommand>(topicName, 10);
  
  using namespace std::chrono_literals;
  const auto callbackTime = 10ms;
  const auto startTime = sinusoidalNode->now();
  const auto timerCallback = [&]() 
  {
    auto commandMsg = joint_controller_msgs::msg::JointCommand();

    std::vector<std::string> names;
    names.push_back("body_1_joint");

    std::vector<double> positions;
    positions.push_back(0.0);

    std::vector<double> velocities;
    velocities.push_back(0.0);

    std::vector<double> torques;
    torques.push_back(0.0);

    std::vector<double> kpScales;
    kpScales.push_back(0.0);

    std::vector<double> kdScales;
    kdScales.push_back(0.0);

    commandMsg.desired_position = positions;
    commandMsg.desired_velocity = velocities;
    commandMsg.feedforward_effort = torques;
    commandMsg.name = names;
    commandMsg.kp_scale = kpScales;
    commandMsg.kd_scale = kdScales;

    const auto nowTime = sinusoidalNode->now();
    const auto duration = nowTime - startTime;
    const double seconds = duration.nanoseconds() / 1e+9;
    const auto sinValue = amplitude * sin(angularVelocity * seconds);
    commandMsg.desired_position[0] = sinValue;
    sinusPublisher->publish(commandMsg);
  };
  const auto timer = sinusoidalNode->create_wall_timer(callbackTime,timerCallback);

  rclcpp::spin(sinusoidalNode);
  rclcpp::shutdown();
  return 0;
}