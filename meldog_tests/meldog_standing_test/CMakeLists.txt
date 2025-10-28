cmake_minimum_required(VERSION 3.8)
project(meldog_standing_test)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)

# Resolve for the package path at compile time.
configure_file (
  "${PROJECT_SOURCE_DIR}/include/${PROJECT_NAME}/path_management/package_path.h.in"
  "${PROJECT_SOURCE_DIR}/include/${PROJECT_NAME}/path_management/package_path.h" @ONLY
)

##################
## Dependencies ##
##################

find_package(ament_cmake REQUIRED)
set(THIS_PACKAGE_DEPENDS
  multi_end_effector_kinematics

  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  tf2_ros
)

foreach(Dependency IN ITEMS ${THIS_PACKAGE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

add_executable(base_controller
  src/main.cpp
  src/BaseControllerNode.cpp
  src/core/BaseController.cpp
)

target_include_directories(base_controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)

ament_target_dependencies(base_controller ${THIS_PACKAGE_DEPENDS})

install(TARGETS base_controller
    DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY bringup
  DESTINATION share/${PROJECT_NAME}
)

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

ament_export_dependencies(${THIS_PACKAGE_DEPENDS})
ament_package()
