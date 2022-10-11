// 包含rclcpp头文件，如果Vscode显示红色的波浪线也没关系
// 我们只是把VsCode当记事本而已，谁会在意记事本对代码的看法呢，不是吗？

// ls /opt/ros/humble/lib | grep rclcpp
// ls /opt/ros/humble/include/rclcpp/* |grep rclcpp.h
// -I head
// undefined reference to xxxxx  g++找不到库文件  -L dir -l file


//
// g++ first_ros2_node.cpp \
// -I/opt/ros/humble/include/rclcpp/ \
// -I /opt/ros/humble/include/rcl/ \
// -I /opt/ros/humble/include/rcutils/ \
// -I /opt/ros/humble/include/rmw \
// -I /opt/ros/humble/include/rcl_yaml_param_parser/ \
// -I /opt/ros/humble/include/rosidl_runtime_c \
// -I /opt/ros/humble/include/rosidl_typesupport_interface \
// -I /opt/ros/humble/include/rcpputils \
// -I /opt/ros/humble/include/builtin_interfaces \
// -I /opt/ros/humble/include/rosidl_runtime_cpp \
// -I /opt/ros/humble/include/tracetools \
// -I /opt/ros/humble/include/rcl_interfaces \
// -I /opt/ros/humble/include/libstatistics_collector \
// -I /opt/ros/humble/include/statistics_msgs \
// -L /opt/ros/humble/lib/ \
// -lrclcpp -lrcutils
//

#include "rclcpp/rclcpp.hpp"

int main(int argc, char **argv)
{
    // 调用rclcpp的初始化函数
    rclcpp::init(argc, argv);
    // 调用rclcpp的循环运行我们创建的first_node节点
    rclcpp::spin(std::make_shared<rclcpp::Node>("first_node"));
    return 0;
}

