################################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 3.5)
project(tf2_rclcpp_example)

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

################################################################################
# Find and load build settings from external packages
################################################################################
find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

################################################################################
# Build
################################################################################
include_directories(
  include
)

set(dependencies
  geometry_msgs
  rclcpp
  std_srvs
  tf2
  tf2_geometry_msgs
  tf2_ros
)

add_executable(broadcaster src/broadcaster.cpp)
ament_target_dependencies(broadcaster ${dependencies})

add_executable(listener src/listener.cpp)
ament_target_dependencies(listener ${dependencies})

add_executable(static_broadcaster src/static_broadcaster.cpp)
ament_target_dependencies(static_broadcaster ${dependencies})

################################################################################
# Install
################################################################################
install(TARGETS
  broadcaster
  listener
  static_broadcaster
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY rviz
  DESTINATION share/${PROJECT_NAME}
)

################################################################################
# Test
################################################################################
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

################################################################################
# Macro for ament package
################################################################################
ament_package()
