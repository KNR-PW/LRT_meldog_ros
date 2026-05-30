#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  auto node = rclcpp::Node::make_shared("wrench_node");

  auto wrenchPublisher = node->create_publisher<geometry_msgs::msg::WrenchStamped>(
    "baseWrench", 10);

  rclcpp::Rate loop_rate(std::chrono::nanoseconds(100));

  geometry_msgs::msg::WrenchStamped wrenchMsg;

  wrenchMsg.header.frame_id = "trunk_link";
  wrenchMsg.header.stamp = node->now();

  wrenchMsg.wrench.force.x = 30;
  wrenchMsg.wrench.force.y = 50;
  wrenchMsg.wrench.force.z = 100;

  wrenchMsg.wrench.torque.x = 60;
  wrenchMsg.wrench.torque.y = 20;
  wrenchMsg.wrench.torque.z = 40;

  while(rclcpp::ok())
  {
    wrenchPublisher->publish(wrenchMsg);
    rclcpp::spin_some(node);
    loop_rate.sleep();
  }

  rclcpp::shutdown();
  return 0;
}