#include <meldog_standing_test/BaseControllerNode.hpp>


int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);

  multi_end_effector_kinematics::KinematicsModelSettings modelSettings;
  multi_end_effector_kinematics::InverseSolverSettings solverSettings;

  modelSettings.baseLinkName = "trunk_link";
  std::string rightForwardFeet = "RFF_link";
  std::string leftForwardFeet = "LFF_link";
  std::string rightRearFeet = "RRF_link";
  std::string leftRearFeet = "LRF_link";
  std::vector<std::string> threeDofLinks{rightForwardFeet, rightRearFeet, 
    leftForwardFeet, leftRearFeet};
  modelSettings.threeDofEndEffectorNames = threeDofLinks;
  

  multi_end_effector_kinematics::MultiEndEffectorKinematics kinematicsSolver(
    "/home/bartek/meldog_base_link.urdf", modelSettings, solverSettings, "NewtonRaphson");
  meldog_standing_test::BaseController::Settings settings;
  settings.minimumHeight = 0.05;
  settings.maximumHeight = 1.0;
  settings.deltaTime = 0.5;
  auto baseControllerNode = std::make_shared<meldog_standing_test::BaseControllerNode>(
    settings, std::move(kinematicsSolver));

  rclcpp::executors::SingleThreadedExecutor executor;

  executor.add_node(baseControllerNode);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}