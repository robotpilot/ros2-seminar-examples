# ROS 2 example packages for the ROS 2 seminar

| Travis CI (master)  | Travis CI (develop) | Ament Lint |
| ------------- | ------------- | ------------- |
| [![Build Status](https://travis-ci.com/robotpilot/ros2-seminar-examples.svg?branch=main)](https://travis-ci.com/github/robotpilot/ros2-seminar-examples)  | [![Build Status](https://travis-ci.com/robotpilot/ros2-seminar-examples.svg?branch=develop)](https://travis-ci.com/github/robotpilot/ros2-seminar-examples)  |  [![Lint](https://github.com/robotpilot/ros2-seminar-examples/workflows/Lint/badge.svg?branch=develop)](https://github.com/robotpilot/ros2-seminar-examples/actions) |

## ROS 2 online seminar
- https://cafe.naver.com/openrt/24070

## GitHub Action for ROS Lint
- `.github/workflows/lint.yml`

## Travis option for CI
- `.travis.yml`

## ROS 2 Distro
- [ROS 2 Foxy Fitzroy - Patch Release 4](https://github.com/ros2/ros2/releases/tag/release-foxy-20201211)

## Build
```
$ cd ~/robot_ws/src
$ git clone https://github.com/robotpilot/ros2-seminar-examples.git
$ cd ~/robot_ws && colcon build --symlink-install
```

## Run time_rclcpp_example package
```
$ ros2 run time_rclcpp_example time_example
```

## Run my_first_ros_rclpy_pkg package
```
$ ros2 run my_first_ros_rclpy_pkg helloworld_subscriber
$ ros2 run my_first_ros_rclpy_pkg helloworld_publisher
```

## Run my_first_ros_rclcpp_pkg package
```
$ ros2 run my_first_ros_rclcpp_pkg helloworld_subscriber
$ ros2 run my_first_ros_rclcpp_pkg helloworld_publisher
```

## Run topic_service_action_rclpy_example package
```
$ ros2 run topic_service_action_rclpy_example argument
$ ros2 run topic_service_action_rclpy_example operator
$ ros2 run topic_service_action_rclpy_example calculator
$ ros2 run topic_service_action_rclpy_example checker
$ ros2 run topic_service_action_rclpy_example checker -g 100
$ ros2 launch topic_service_action_rclpy_example arithmetic.launch.py
```

## Run topic_service_action_rclcpp_example package
```
$ ros2 run topic_service_action_rclcpp_example argument
$ ros2 run topic_service_action_rclcpp_example operator
$ ros2 run topic_service_action_rclcpp_example calculator
$ ros2 run topic_service_action_rclcpp_example checker
$ ros2 run topic_service_action_rclcpp_example checker -g 100.0
$ ros2 launch topic_service_action_rclcpp_example arithmetic.launch.py
```

## Run ros2env package
```
$ ros2 env
usage: ros2 env [-h] Call `ros2 env <command> -h` for more detailed usage. ...
Various env related sub-commands
optional arguments:
  -h, --help            show this help message and exit
Commands:
  list  Output a list of ROS environment variables
  set   Set ROS environment variables

$ ros2 env list -a
ROS_VERSION        = 2
ROS_DISTRO         = foxy
ROS_PYTHON_VERSION = 3
ROS_DOMAIN_ID      = 7
RMW_IMPLEMENTATION = rmw_fastrtps_cpp
```

## Run rqt_example package
```
ros2 run rqt_example rqt_example
ros2 launch rqt_example turtlesim.launch.py
```

## Run logging_rclpy_example package
```
$ ros2 run logging_rclpy_example logging_example
```
