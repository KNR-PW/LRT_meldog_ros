
cmake_minimum_required(VERSION 3.8)
project(meldog_description)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY description config launch meshes rviz controllers worlds
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_description test/test_description.py
    TIMEOUT 120
  )
endif()

ament_package()
