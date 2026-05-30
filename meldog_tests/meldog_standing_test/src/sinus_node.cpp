#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include <std_msgs/msg/float64.hpp>

const double minimumHeight = 0.1;
const double maximumHeight = 0.5;

class sinusNode: public rclcpp::Node
{
  public:
    sinusNode(): rclcpp::Node("sinusNode")
    {
      sinusPublisher_ = create_publisher<std_msgs::msg::Float64>(
        "/base_controller/target_base_height", 1);

      std::chrono::duration<double> timerPeriod(0.1);
      controldTimer_ = create_wall_timer(timerPeriod, std::bind(&sinusNode::sinusCallback, 
        this));

      startTime_ = now();
    }

    double sinusCallback()
    {
      
    };

  private:
    rclcpp::Time startTime_(0, 0, RCL_ROS_TIME);
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr sinusPublisher_;
    rclcpp::TimerBase::SharedPtr controldTimer_;
}



int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  auto sinusNode = std::make_shared<rclcpp::Node>("sinusNode");

  sinusNode->

  rclcpp::executors::SingleThreadedExecutor executor;

  executor.add_node(sinusNode);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}