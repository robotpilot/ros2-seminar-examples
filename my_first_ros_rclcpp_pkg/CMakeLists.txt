# Set minimum required version of cmake, project name and compile options
cmake_minimum_required(VERSION 3.5)
project(my_first_ros_rclcpp_pkg)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# Build
add_executable(helloworld_publisher src/helloworld_publisher.cpp)
ament_target_dependencies(helloworld_publisher rclcpp std_msgs)

add_executable(helloworld_subscriber src/helloworld_subscriber.cpp)
ament_target_dependencies(helloworld_subscriber rclcpp std_msgs)

# Install
install(TARGETS
  helloworld_publisher
  helloworld_subscriber
  DESTINATION lib/${PROJECT_NAME})

# Test
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# Macro for ament package
ament_package()
