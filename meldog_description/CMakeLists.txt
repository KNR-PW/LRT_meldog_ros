
cmake_minimum_required(VERSION 3.8)
project(meldog_description)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY description launch meshes rviz controllers worlds
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
