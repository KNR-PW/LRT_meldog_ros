#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/spatial/se3.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <string>
#include <vector>
#include <map>
#include <chrono>
#include <cmath>

using namespace std::chrono_literals;

class LegController : public rclcpp::Node
{
public:
  LegController() : Node("leg_trajectory_node")
  {
    std::string package_path = ament_index_cpp::get_package_share_directory("meldog_leg_description");
    std::string urdf_path = package_path + "/description/meldog_core.urdf";
    
    try {
      pinocchio::urdf::buildModel(urdf_path, model_);
    } catch (const std::exception& e) {
      RCLCPP_FATAL(this->get_logger(), "Błąd ładowania URDF: %s", e.what());
      rclcpp::shutdown();
      return;
    }
    
    data_ = pinocchio::Data(model_);
    initializeJoints();

    foot_frame_id_ = model_.getFrameId("LRF_link");
    if (foot_frame_id_ == model_.nframes) {
      RCLCPP_FATAL(this->get_logger(), "Brak ramki 'LRF_link' w modelu");
      rclcpp::shutdown();
      return;
    }

    q_current_ = pinocchio::neutral(model_);

    joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", 10,
      [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
        for (size_t i = 0; i < msg->name.size(); ++i) {
          const auto& name = msg->name[i];
          if (model_.existJointName(name)) {
            const auto& joint = model_.getJointId(name);
            const auto idx_q = model_.joints[joint].idx_q();
            if (idx_q + model_.joints[joint].nq() <= model_.nq) {
              q_current_.segment(idx_q, model_.joints[joint].nq()) = 
                Eigen::Map<const Eigen::VectorXd>(&msg->position[i], model_.joints[joint].nq());
            }
          }
        }
      });

    traj_pub_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>("foot_trajectory", 10);
    timer_ = this->create_wall_timer(20ms, std::bind(&LegController::updateTrajectory, this));
  }
private:

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr traj_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  pinocchio::Model model_;
  pinocchio::Data data_;
  pinocchio::FrameIndex foot_frame_id_;

  std::map<std::string, int> joint_index_map_;
  std::vector<std::string> active_joint_names_;

  Eigen::VectorXd q_current_;
  rclcpp::Time t_start_;

  void initializeJoints()
  {
    active_joint_names_.clear();
    joint_index_map_.clear();
    
    for (const auto & joint_name : model_.names)
    {
      int joint_id = model_.getJointId(joint_name);

      if (joint_id > 0 && model_.joints[joint_id].nq() > 0)
      {
        active_joint_names_.push_back(joint_name);
        joint_index_map_[joint_name] = joint_id;
      }
    }

    RCLCPP_INFO(this->get_logger(), "Znalezione stawy:");
    for (const auto & name : active_joint_names_) {
      RCLCPP_INFO(this->get_logger(), "- %s", name.c_str());
    }
  }

  void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    for (size_t i = 0; i < msg->name.size(); ++i)
    {
      const auto & joint_name = msg->name[i];
      if (joint_index_map_.count(joint_name))
      {
        int joint_id = joint_index_map_[joint_name];
        if (joint_id < q_current_.size())
        {
          q_current_[joint_id] = msg->position[i];
        }
      }
    }
  }


  bool computeIK(const pinocchio::SE3& desired_pose, Eigen::VectorXd& q)
  {
    const int max_iter = 100;
    const double tol = 1e-4;
    const double damping = 1e-6;
    const double alpha = 0.1; // Współczynnik kroku

    for (int iter = 0; iter < max_iter; ++iter)
    {
      pinocchio::forwardKinematics(model_, data_, q);
      pinocchio::updateFramePlacement(model_, data_, foot_frame_id_);
      
      const pinocchio::SE3 current_pose = data_.oMf[foot_frame_id_];
      const pinocchio::SE3 error_se3 = desired_pose * current_pose.inverse();
      const Eigen::Matrix<double, 6, 1> error = pinocchio::log6(error_se3).toVector();

      if (error.norm() < tol) return true;

      Eigen::MatrixXd J(6, model_.nv);
      J.setZero();
      pinocchio::computeFrameJacobian(
        model_, data_, q, foot_frame_id_,
        pinocchio::ReferenceFrame::WORLD, J
      );

      // DLS z ograniczeniem prędkości
      const Eigen::MatrixXd Jt = J.transpose();
      const Eigen::MatrixXd JJt = J * Jt + damping * Eigen::MatrixXd::Identity(6, 6);
      const Eigen::VectorXd delta_q = alpha * Jt * JJt.ldlt().solve(error);
      
      q = pinocchio::integrate(model_, q, delta_q);
    }
    RCLCPP_WARN(this->get_logger(), "IK nie zbiegło się");
    return false;
  }

  void updateTrajectory()
  {
    static Eigen::VectorXd last_q = q_current_; // Ciepły start
    const double t = (this->now() - t_start_).seconds();
    
    constexpr double radius = 0.05;
    constexpr double omega = 2.0;
    constexpr double z_offset = -0.15;

    pinocchio::SE3 target_pose(
      Eigen::Matrix3d::Identity(),
      Eigen::Vector3d(
        radius * std::cos(omega * t),
        radius * std::sin(omega * t),
        z_offset
      )
    );

    Eigen::VectorXd q_target = last_q;
    if (!computeIK(target_pose, q_target)) {
      RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "IK failed");
      return;
    }
    last_q = q_target;

    trajectory_msgs::msg::JointTrajectory traj_msg;
    traj_msg.header.stamp = this->now();
    traj_msg.joint_names = active_joint_names_;

    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.positions.reserve(active_joint_names_.size());

    for (const auto& name : active_joint_names_) {
      const auto& joint_id = model_.getJointId(name);
      const auto idx_q = model_.joints[joint_id].idx_q();
      point.positions.push_back(q_target[idx_q]);
    }

    point.time_from_start = rclcpp::Duration::from_seconds(0.02);
    traj_msg.points.push_back(point);

    traj_pub_->publish(traj_msg);
  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LegController>());
  rclcpp::shutdown();
  return 0;
}