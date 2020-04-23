#include <cstdio>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"

using std::placeholders::_1;

typedef enum {
  LEFT,
  RIGHT
} HBSide;

class HadabotDriver : public rclcpp::Node
{
  private:

    void wheel_radps(
      const std_msgs::msg::Float32::SharedPtr msg, HBSide which_side)
    {
      std::string the_side = which_side == HBSide::LEFT ? "left" : "right";
      RCLCPP_INFO(
        this->get_logger(), "%s side: '%f'", the_side.c_str(), msg->data);
    }

    void wheel_radps_left_cb(const std_msgs::msg::Float32::SharedPtr msg)
    {
      this->wheel_radps(msg, HBSide::LEFT);
    }

    void wheel_radps_right_cb(const std_msgs::msg::Float32::SharedPtr msg)
    {
      this->wheel_radps(msg, HBSide::RIGHT);
    }

    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr radps_left_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr radps_right_sub_;

  public:

    HadabotDriver() : Node("hadabot_driver") 
    {
      radps_left_sub_ = this->create_subscription<std_msgs::msg::Float32>(
        "/hadabot/wheel_radps_left", 10,
        std::bind(&HadabotDriver::wheel_radps_left_cb, this, _1)
      );

      radps_right_sub_ = this->create_subscription<std_msgs::msg::Float32>(
        "/hadabot/wheel_radps_right", 10,
        std::bind(&HadabotDriver::wheel_radps_right_cb, this, _1)
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
