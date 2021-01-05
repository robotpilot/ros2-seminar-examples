# ros2-seminar-examples

## GitHub Action for ROS Lint
- `.github/workflows/lint.yml`

## Build
```
$ cd ~/robot_ws/src
$ git clone https://github.com/robotpilot/ros2-seminar-examples.git
$ cd ~/robot_ws && colcon build --symlink-install
```

## Run (my_first_ros_rclpy_pkg)
```
$ ros2 run my_first_ros_rclpy_pkg helloworld_subscriber
```

```
$ ros2 run my_first_ros_rclpy_pkg helloworld_publisher
```

## Run (my_first_ros_rclcpp_pkg)
```
$ ros2 run my_first_ros_rclcpp_pkg helloworld_subscriber
```

```
$ ros2 run my_first_ros_rclcpp_pkg helloworld_publisher
```

## Run (topic_service_action_rclpy_example)
### Topic Publisher (argument node)
```
$ ros2 run topic_service_action_rclpy_example argument
```
### Service Client (operator node)
```
$ ros2 run topic_service_action_rclpy_example operator
```
### Topic subscriber, Service Server, Action Server (calculator node)
```
$ ros2 run topic_service_action_rclpy_example calculator
```
### Action Client (checker node)
```
$ ros2 run topic_service_action_rclpy_example checker
```
```
$ ros2 run topic_service_action_rclpy_example checker -g 100
```
### Run using launch (argument and calculator nodes)
```
$ ros2 launch topic_service_action_rclpy_example arithmetic.launch.py
```

## Run (topic_service_action_rclcpp_example)
### Topic Publisher (argument node)
```
$ ros2 run topic_service_action_rclcpp_example argument
```
### Service Client (operator node)
```
$ ros2 run topic_service_action_rclcpp_example operator
```
### Topic subscriber, Service Server, Action Server (calculator node)
```
$ ros2 run topic_service_action_rclcpp_example calculator
```
### Action Client (checker node)
```
$ ros2 run topic_service_action_rclcpp_example checker
```
```
$ ros2 run topic_service_action_rclcpp_example checker -g 100.0
```
### Run using launch (argument and calculator nodes)
```
$ ros2 launch topic_service_action_rclcpp_example arithmetic.launch.py
```
