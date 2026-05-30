#include <meldog_standing_test/core/BaseController.hpp>

#include <algorithm>
#include <stdexcept>

namespace meldog_standing_test
{
  using namespace multi_end_effector_kinematics;

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  BaseController::BaseController(BaseController::Settings settings, 
    MultiEndEffectorKinematics&& kinematicsSolver): settings_(std::move(settings)), 
      kinematicsSolver_(std::move(kinematicsSolver)) {}
  
  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  Eigen::VectorXd BaseController::alignJoints(const Eigen::VectorXd& currentPositions)
  {
    Eigen::VectorXd newPositions = currentPositions;
    std::vector<size_t> visitedIndexes;
    for(size_t i = 0; i < newPositions.size(); ++i)
    {
      const auto currentIterator = std::find(visitedIndexes.begin(), visitedIndexes.end(), i);
      if(currentIterator != visitedIndexes.end())
      {
        visitedIndexes.push_back(i);
      }
      else
      {
        continue;
      }
      for(size_t j = 0; j < newPositions.size(); ++j)
      {
        const auto nextIterator = std::find(visitedIndexes.begin(), visitedIndexes.end(), j);
        if(nextIterator != visitedIndexes.end())
        {
          if(std::abs(newPositions[i] - newPositions[j]) < 
            settings_.jointDifferencePositionThreshold)
          {
            newPositions[j] = newPositions[i];
            visitedIndexes.push_back(j);
          }
        }
      }
    }
    return newPositions;
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  void BaseController::updateCurrentHeight(const Eigen::VectorXd& currentPositions)
  {
    std::vector<Eigen::Vector3d> endEffectorPositions(LEG_NUMBER);
    kinematicsSolver_.calculateEndEffectorPoses(currentPositions, endEffectorPositions);

    double meanHeight = 0;
    for(size_t i = 0; i < LEG_NUMBER; ++i)
    {
      /**
       *  Base link is at height 0, so end effectors have negative z coordinate
       */ 
      meanHeight  += -endEffectorPositions[i].z();;
    }

    currentHeight_ = meanHeight / LEG_NUMBER;
  }
  
  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  using time_joint_positions_t = std::pair<std::vector<double>, std::vector<Eigen::VectorXd>>;
  time_joint_positions_t BaseController::getJointTrajectory(
    const Eigen::VectorXd& currentPositions, 
    double moveTime, 
    double newHeight)
  {
    Eigen::VectorXd actualJointPosition = currentPositions;
    Eigen::VectorXd nextJointPosition;

    std::vector<Eigen::Vector3d> endEffectorPositions(LEG_NUMBER);
    kinematicsSolver_.calculateEndEffectorPoses(currentPositions, endEffectorPositions);
    
    double clampedHeight = std::clamp(newHeight, settings_.minimumHeight, 
      settings_.maximumHeight);

    const size_t trajectorySize = moveTime / settings_.deltaTime + 1;
    
    std::vector<double> times;
    times.reserve(trajectorySize);

    std::vector<Eigen::VectorXd> positions;
    positions.reserve(trajectorySize);

    times.push_back(0.0);
    positions.push_back(currentPositions);

    const double jumpHeight = (clampedHeight - currentHeight_) / (trajectorySize  - 1);
    for(size_t i = 1; i < trajectorySize; ++i)
    {
      const double nextTime = i * settings_.deltaTime;

      for(size_t j = 0; j < LEG_NUMBER; ++j)
      {
        /**
         *  Base link is at height 0, so end effectors have negative z coordinate
         */ 
        endEffectorPositions[j].z() -= jumpHeight;
      }

      const auto result = kinematicsSolver_.calculateJointPositions(actualJointPosition, 
        endEffectorPositions, nextJointPosition);
      
      if(!result.success)
      {
        throw std::runtime_error("Error: " + returnFlagToString(result.flag));
      }

      times.push_back(nextTime);
      positions.push_back(nextJointPosition);

      actualJointPosition = nextJointPosition;
    }
    return {times, positions};
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  double BaseController::getCurrentHeight()
  {
    return currentHeight_;
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  std::vector<std::string> BaseController::getJointNames()
  {
    std::vector<std::string> names = kinematicsSolver_.getPinocchioModel().names;
    names.erase(std::remove(names.begin(), names.end(), 
      "universe"), names.end());
    return names;
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  const BaseController::Settings& BaseController::getSettings()
  {
    return settings_;
  }
} // namespace meldog_standing_test
