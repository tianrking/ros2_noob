
rosdep install -i --from-path src --rosdistro humble -y

colcon build --packages-select cpp_srvcli

ros2 run cpp_srvcli server

ros2 run cpp_srvcli client 2 3