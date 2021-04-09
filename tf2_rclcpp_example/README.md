# ROS Packages for reference (examples_tf)

## Run examples_tf
```bash
$ ros2 run examples_tf2 broadcaster
$ ros2 run examples_tf2 listener
$ ros2 run examples_tf2 static_broadcaster
```

```bash
$ ros2 run rviz2 rviz2 -d examples_tf2/rviz/arm.rviz
```

```bash
$ ros2 service call state std_srvs/srv/SetBool "data: false"
```
