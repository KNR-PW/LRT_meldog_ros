#include <meldog_standing_test/BaseControllerNode.hpp>

namespace meldog_standing_test
{
  using namespace rclcpp;

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  BaseControllerNode::BaseControllerNode(BaseController::Settings settings, 
    multi_end_effector_kinematics::MultiEndEffectorKinematics&& kinematicsSolver):
      Node("base_controller"), baseController_(settings, std::move(kinematicsSolver))
  {

    declare_parameter("joint_names", std::vector<std::string>());
    get_parameter("joint_names", jointNamesInControllers_);

    heightSubscriber_ = create_subscription<std_msgs::msg::Float64>(
      "base_controller/target_base_height", 1, 
      std::bind(&BaseControllerNode::heightCallback, this, std::placeholders::_1));

    jointStateSubscriber_ = create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", 1, 
      std::bind(&BaseControllerNode::jointStateCallback, this, std::placeholders::_1));

    jointCommandPublisher_ = create_publisher<std_msgs::msg::Float64MultiArray>(
      "/forward_position_controller/commands", 1);

    std::chrono::duration<double> timerPeriod(controlLoopPeriod_);
    controldTimer_ = create_wall_timer(timerPeriod, 
      std::bind(&BaseControllerNode::controlLoopCallback, this));

    const auto jointModelNames = baseController_.getJointNames();

    if(jointNamesInControllers_.size() != jointModelNames.size())
    {
      RCLCPP_FATAL(get_logger(), "Wrong number of joint names given!");
      throw std::runtime_error("BaseControllerNode: Wrong number of joint names given!");
    }

    for(size_t i = 0; i < jointModelNames.size(); ++i)
    {
      jointModelNameToIndexMap_.emplace(jointModelNames[i], i);
    }

    state_ = BaseControllerNode::ControlState::CALIBRATION;

    currentJointPositions_ = Eigen::VectorXd(jointModelNames.size());
    RCLCPP_INFO(get_logger(), "Base controller initialized!");
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  void BaseControllerNode::heightCallback(std_msgs::msg::Float64::UniquePtr message)
  {
    if(state_ != BaseControllerNode::ControlState::WAITING_FOR_TARGET)
    {
      return;
    }

    targetHeight_ = message->data;
    try
    {
      RCLCPP_INFO(get_logger(), "Inverse kinematics starting!");
      const auto startTime = now();
      targetTrajectory_ = baseController_.getJointTrajectory(currentJointPositions_, 
        targetTime_, targetHeight_);
      const auto duration = now() - startTime;
      RCLCPP_INFO(get_logger(), "Inverse kinematics ended, computation time [s]: %f", 
        duration.seconds());
      std::vector<double>& deltaTimes = targetTrajectory_.first;
      const auto currentTime = now().seconds();
      for(auto& time : deltaTimes)
      {
        time += currentTime;
      }
      RCLCPP_INFO(get_logger(), "New trajectory starting!");
      state_ = BaseControllerNode::ControlState::SENDING_DATA;
    }
    catch(const std::exception& e)
    {
      RCLCPP_FATAL(get_logger(), "%s\n", e.what());
      throw; 
    }
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  void BaseControllerNode::jointStateCallback(
    sensor_msgs::msg::JointState::SharedPtr message)
  {
    static rclcpp::Time previousCllbackTime = now();

    for(size_t i = 0; i < message->position.size(); ++i)
    {
      currentJointPositions_[jointModelNameToIndexMap_[message->name[i]]] = message->position[i];
    }

    baseController_.updateCurrentHeight(currentJointPositions_);

    if((now() - previousCllbackTime) > rclcpp::Duration(1, 0))
    {
      const double currentHeight = baseController_.getCurrentHeight();
      RCLCPP_INFO(get_logger(), "Current base height: %f", currentHeight);
      previousCllbackTime = now();
    }

    if(state_ == BaseControllerNode::ControlState::CALIBRATION)
    {
      currentJointPositions_ = baseController_.alignJoints(currentJointPositions_);
      publishJointPositions(currentJointPositions_);
      state_ = BaseControllerNode::ControlState::WAITING_FOR_TARGET;
    }
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  void BaseControllerNode::publishJointPositions(const Eigen::VectorXd& jointPositions)
  {
    std::vector<double> positionData(jointPositions.size());
    for(size_t i = 0; i < jointPositions.size(); ++i)
    {
      const size_t modelIndex = jointModelNameToIndexMap_[jointNamesInControllers_[i]];
      positionData[i] = jointPositions[modelIndex];
    }
    std_msgs::msg::Float64MultiArray message;
    message.data = std::move(positionData);
    
    jointCommandPublisher_->publish(message);
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  void BaseControllerNode::controlLoopCallback()
  {
  
    if(state_ != BaseControllerNode::ControlState::SENDING_DATA)
    {
      return;
    }

    const std::vector<double>& times = targetTrajectory_.first;
    const std::vector<Eigen::VectorXd>& positions = targetTrajectory_.second;

    const double currentTime = now().seconds();
    const auto queryPositionIterator = std::upper_bound(times.begin(), times.end(), 
      currentTime);
    
    size_t queryPositionIndex;
    if(queryPositionIterator != times.end())
    {
      const size_t queryPositionIndex = std::distance(times.begin(), 
        queryPositionIterator);
      const Eigen::VectorXd newJointPositions = getInterpolatedJointPositions(currentTime,
        times[queryPositionIndex - 1], positions[queryPositionIndex - 1], 
        positions[queryPositionIndex]);
        publishJointPositions(newJointPositions);
    }
    else
    {
      RCLCPP_INFO(get_logger(), "Trajectory sent!");
      state_ = BaseControllerNode::ControlState::WAITING_FOR_TARGET;
    }
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  Eigen::VectorXd BaseControllerNode::getInterpolatedJointPositions(double currentTime,
    double previousTime, const Eigen::VectorXd& previousJointPositions,
    const Eigen::VectorXd& nextJointPositions)
  {
    return previousJointPositions + (nextJointPositions - previousJointPositions) / 
      baseController_.getSettings().deltaTime * (currentTime - previousTime);
    
  }
} // namespace meldog_standing_test
