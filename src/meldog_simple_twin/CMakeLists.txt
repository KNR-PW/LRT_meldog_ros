cmake_minimum_required(VERSION 3.8)
project(meldog_simple_twin)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY urdf launch meshes rviz
  DESTINATION share/${PROJECT_NAME}
)


ament_package()
