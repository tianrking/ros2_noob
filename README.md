# NEED INSTALL

```bash
sudo apt-get install python3-colcon-common-extensions
```


## NOTE

### Create package & node

```bash
cd chapt3/chapt3_ws/src
ros2 pkg create example_service_rclcpp --build-type ament_cmake --dependencies rclcpp
```

### 手动调用服务

```bash
ros2 run examples_rclpy_minimal_service service
```

```bash
ros2 service list
```

```bash
ros2 service call /add_two_ints example_interfaces/srv/AddTwoInts "{a: 5,b: 10}"
```

```bash
ros2 service type /add_two_ints
```

```bash
ros2 service find example_interfaces/srv/AddTwoInts
```

```bash
ros2 interface show example_interfaces/srv/AddTwoInts
```

## Todo

[example_interfaces_rclcpp](https://fishros.com/d2lros2/#/humble/chapt3/get_started/8.%E8%87%AA%E5%AE%9A%E4%B9%89%E6%8E%A5%E5%8F%A3RCLCPP%E5%AE%9E%E6%88%98) cant work well



