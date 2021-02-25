#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

class IntroNode : public rclcpp::Node
{
public:
  IntroNode()
      : Node("intro_ros2_node_cpp"), count_(0)
  {
    timer_ = this->create_wall_timer(
        500ms, std::bind(&IntroNode::timer_callback, this));
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
  rclcpp::spin(std::make_shared<IntroNode>());
  rclcpp::shutdown();
  return 0;
}
