# ros2-seminar-examples

## GitHub Action for ROS Lint
- `.github/workflows/lint.yml`

## Build
```
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
