#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
using std::placeholders::_1;

class MySubscriberNode : public rclcpp::Node
{
public:
  MySubscriberNode()
      : Node("intro_ros2_topic_subscriber_cpp")
  {
    RCLCPP_INFO(this->get_logger(), "C++ node waiting for a ROS 2 message...");

    // Add subscriber code here
  }

private:
  void topic_callback(const std_msgs::msg::String::SharedPtr msg) const
  {
    RCLCPP_INFO(this->get_logger(), "ROS 2 message received by C++ node: '%s'", msg->data.c_str());
  }
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto my_subscriber_node = std::make_shared<MySubscriberNode>();
  rclcpp::spin(my_subscriber_node);
  rclcpp::shutdown();
  return 0;
}
