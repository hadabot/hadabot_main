#include <cstdio>
#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "nav_msgs/msg/odometry.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

typedef enum {
  LEFT,
  RIGHT
} HBSide;

#define UPDATE_DT 100ms
#define PUBLISH_DT 500ms

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

    void update_odometry() 
    {
      // Get the time delta since last update
      //int dt_ms;

      // Compute distance traveled for each wheel
      //float d_left_m;
      //float d_right_m;

    }

    void publish_odometry()
    {
      rclcpp::Clock clk = rclcpp::Clock();
      double t = clk.now().seconds();
      RCLCPP_INFO(this->get_logger(), "Time: %f", t); 	
    }

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_pub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr radps_left_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr radps_right_sub_;
    rclcpp::TimerBase::SharedPtr update_odometry_timer_;
    rclcpp::TimerBase::SharedPtr publish_odometry_timer_;

    float wheel_radius_m_;
    float wheelbase_m_;

  public:

    HadabotDriver() : Node("hadabot_driver"), wheel_radius_m_(0.035), wheelbase_m_(0.14)
    {
      radps_left_sub_ = this->create_subscription<std_msgs::msg::Float32>(
        "/hadabot/wheel_radps_left", 10,
        std::bind(&HadabotDriver::wheel_radps_left_cb, this, _1)
      );

      radps_right_sub_ = this->create_subscription<std_msgs::msg::Float32>(
        "/hadabot/wheel_radps_right", 10,
        std::bind(&HadabotDriver::wheel_radps_right_cb, this, _1)
      );

      odometry_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(
        
      )

      publish_odometry_timer_ = this->create_wall_timer(
        PUBLISH_DT, std::bind(&HadabotDriver::publish_odometry, this));
    }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<HadabotDriver>());
  rclcpp::shutdown();
  return 0;
}
