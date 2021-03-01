#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

class MyROSNode : public rclcpp::Node
{
public:
  MyROSNode()
      : Node("intro_ros2_node_cpp"), count_(0)
  {
    // Add timer code here
  }

private:
  void timer_callback()
  {
    std::string message = "Hello from the Hadabot ROS 2 intro C++ node: " + std::to_string(count_++);
    RCLCPP_INFO(this->get_logger(), "'%s'", message.c_str());
  }
  rclcpp::TimerBase::SharedPtr timer_;
  size_t count_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto my_ros_node = std::make_shared<MyROSNode>();
  rclcpp::spin(my_ros_node);
  rclcpp::shutdown();
  return 0;
}
