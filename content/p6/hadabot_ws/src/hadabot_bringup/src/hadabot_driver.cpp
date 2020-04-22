#include <cstdio>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"

using std::placeholders::_1;

typedef enum {
  HB_LEFT,
  HB_RIGHT
} HBSide;

class HadabotDriver : public rclcpp::Node
{
  private:

  void wheel_radps_cb(const std_msgs::msg::Float32::SharedPtr msg)
  {
    RCLCPP_INFO(this->get_logger(), "I heard: '%f'", msg->data);
  }

  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr radps_left_sub_;

  public:

  HadabotDriver() : Node("hadabot_driver") 
  {
    radps_left_sub_ = this->create_subscription<std_msgs::msg::Float32>(
      "/hadabot/wheel_radps_left", 10,
      std::bind(&HadabotDriver::wheel_radps_cb, this, _1)
    );
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<HadabotDriver>());
  rclcpp::shutdown();
  return 0;
}
