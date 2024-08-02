cmake_minimum_required(VERSION 3.8)
project(pi3hat_hardware_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(pluginlib REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(transmission_interface REQUIRED)

add_library(pi3hat_hardware_interface SHARED
  src/pi3hat_hardware_interface.cpp
)

add_library(pi3hat SHARED
  include/3rd_libs/pi3hat/pi3hat.cc
)

add_library(controllers SHARED
  src/controllers/ControllerBridge.cpp
  src/controllers/wrappers/ControllerWrapper.cpp
  src/controllers/wrappers/MoteusWrapper.cpp
)

add_library(imu_transform SHARED
  src/imu_transform/IMUTransform.cpp
)

# add include directory
target_include_directories(pi3hat_hardware_interface
  PRIVATE
    include
)

target_link_libraries(pi3hat_hardware_interface
  pi3hat
  controllers
  imu_transform
  bcm_host
)

ament_target_dependencies(
  pi3hat_hardware_interface
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  transmission_interface
)

pluginlib_export_plugin_description_file(hardware_interface pi3hat_hardware_interface.xml)


# install
install(DIRECTORY
  bringup/launch
  DESTINATION share/${PROJECT_NAME}/
)

install(
  TARGETS ${PROJECT_NAME}
  DESTINATION lib
)

install(
  TARGETS pi3hat
  DESTINATION lib
)

install(
  TARGETS controllers
  DESTINATION lib
)

install(
  TARGETS imu_transform
  DESTINATION lib
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY test/
  DESTINATION test
)

install(
  DIRECTORY test/urdf
  DESTINATION share/${PROJECT_NAME}/
)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()


# exports
ament_export_include_directories(
  include
)
ament_export_libraries(
  ${PROJECT_NAME}
)
ament_export_dependencies(
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  transmission_interface
)

ament_package()
