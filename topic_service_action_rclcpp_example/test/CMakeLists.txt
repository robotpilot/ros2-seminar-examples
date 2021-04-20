ament_add_gtest(test_calculator
  test_calculator.cpp
  ../src/calculator/calculator.cpp)
ament_target_dependencies(test_calculator
  msg_srv_action_interface_example
  rclcpp
  rclcpp_action)
