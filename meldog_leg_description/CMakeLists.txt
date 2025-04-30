cmake_minimum_required(VERSION 3.8)
project(meldog_leg_description)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(joint_controller_msgs REQUIRED)
find_package(joint_controller REQUIRED)

install(DIRECTORY 
  launch
  description
  meshes
  rviz
  controllers
  DESTINATION share/${PROJECT_NAME}
)

ament_package()