#include "rclcpp/rclcpp.hpp"

class ExampleInterfacesControl : public rclcpp::Node {
public:
  // 构造函数,有一个参数为节点名称
  ExampleInterfacesControl(std::string name) : Node(name) {
    RCLCPP_INFO(this->get_logger(), "节点已启动：%s.", name.c_str());
  }

private:
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ExampleInterfacesControl>("example_interfaces_control_01");
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
