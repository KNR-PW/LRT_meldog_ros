// Copyright (c) 2025, Koło Naukowe Robotyków
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.

// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.

/*
 * Authors: Bartłomiej Krajewski (https://github.com/BartlomiejK2)
 */


#ifndef __BASE_CONTROLLER_NODE_MELDOG_STANDING_TEST__
#define __BASE_CONTROLLER_NODE_MELDOG_STANDING_TEST__

#include <meldog_standing_test/core/BaseController.hpp>

#include <rclcpp/rclcpp.hpp>

#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include <sensor_msgs/msg/joint_state.hpp>

namespace meldog_standing_test
{
  class BaseControllerNode: public rclcpp::Node
  {
    public:

      enum class ControlState
      {
        CALIBRATION = 1,
        WAITING_FOR_TARGET = 2,
        SENDING_DATA = 3,
      };

      BaseControllerNode(BaseController::Settings settings, 
        multi_end_effector_kinematics::MultiEndEffectorKinematics&& kinematicsSolver);

    private:

      void heightCallback(std_msgs::msg::Float64::UniquePtr message);

      void jointStateCallback(sensor_msgs::msg::JointState::SharedPtr message);

      void publishJointPositions(const Eigen::VectorXd& jointPositions);

      Eigen::VectorXd getInterpolatedJointPositions(double currentTime,
        double previousTime, const Eigen::VectorXd& previousJointPositions,
        const Eigen::VectorXd& nextJointPositions);

      void controlLoopCallback();
      
      BaseController baseController_;

      Eigen::VectorXd currentJointPositions_;

      using time_joint_positions_t = std::pair<std::vector<double>, std::vector<Eigen::VectorXd>>;
      time_joint_positions_t targetTrajectory_;

      double targetHeight_ = 0.0;
      double targetTime_ = 5.0;

      ControlState state_;

      double controlLoopPeriod_ = 0.01;

      std::unordered_map<std::string, size_t> jointModelNameToIndexMap_;
      std::vector<std::string> jointNamesInControllers_;


      rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr heightSubscriber_;
      rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr jointStateSubscriber_;
      rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr jointCommandPublisher_;
      rclcpp::TimerBase::SharedPtr controldTimer_;
      
  };
}; // namespace meldog_standing_test
#endif