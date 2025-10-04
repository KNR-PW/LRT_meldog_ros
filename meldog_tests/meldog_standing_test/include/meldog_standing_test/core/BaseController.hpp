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


#ifndef __BASE_CONTROLLER_MELDOG_STANDING_TEST__
#define __BASE_CONTROLLER_MELDOG_STANDING_TEST__

#include <vector>

#include <multi_end_effector_kinematics/MultiEndEffectorKinematics.hpp>

namespace meldog_standing_test
{
  class BaseController
  {
    public:

      struct Settings
      {

        double minimumHeight;

        double maximumHeight;

        double deltaTime;

        double jointDifferencePositionThreshold = 0.05;

        double endEffectorDifferencePositionThreshold = 0.01;
      };

      BaseController(Settings settings, 
        multi_end_effector_kinematics::MultiEndEffectorKinematics&& kinematicsSolver);

      Eigen::VectorXd alignJoints(const Eigen::VectorXd& currentPositions);

      void updateCurrentHeight(const Eigen::VectorXd& currentPositions);

      using time_joint_positions_t = std::pair<std::vector<double>, std::vector<Eigen::VectorXd>>;
      time_joint_positions_t getJointTrajectory(const Eigen::VectorXd& currentPositions, 
        double moveTime, 
        double newHeight);

      std::vector<std::string> getJointNames();

      double getCurrentHeight();

      const Settings& getSettings();

    private:
      
      static const size_t LEG_NUMBER = 4;
      double currentHeight_;
      const Settings settings_;
      multi_end_effector_kinematics::MultiEndEffectorKinematics kinematicsSolver_;

  };
}; // namespace meldog_standing_test

#endif