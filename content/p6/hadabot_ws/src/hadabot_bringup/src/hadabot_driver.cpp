#include <cstdio>
#include <chrono>
#include <memory>
#include <cmath>

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

#define PI 3.14159265

class HadabotDriver : public rclcpp::Node
{
  private:

    void wheel_radps(
      const std_msgs::msg::Float32::SharedPtr msg, HBSide which_side)
    {
      std::string the_side = which_side == HBSide::LEFT ? "left" : "right";

      switch(which_side) {
        case HBSide::LEFT:
        wheel_radps_left_ = msg->data;
        break;

        case HBSide::RIGHT:
        wheel_radps_right_ = msg->data;
        break;
      }
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
      auto dt_ms = UPDATE_DT;
      auto dt_s = dt_ms.count() / 1000.0;

      // Compute distance traveled for each wheel
      float d_left_m = (wheel_radps_left_ * dt_s * wheel_radius_m_) / PI;
      float d_right_m = (wheel_radps_right_ * dt_s * wheel_radius_m_) / PI;

      auto d_center_m = (d_right_m + d_left_m) / 2.0;
      auto phi_rad = (d_right_m - d_left_m) / wheelbase_m_;

      auto theta_rad = pose_->pose.pose.orientation.z;
      auto x_m_dt = d_center_m * std::cos(theta_rad);
      auto y_m_dt = d_center_m * std::sin(theta_rad);
      auto theta_rad_dt = phi_rad;

      auto x_m = pose_->pose.pose.position.x;
      auto y_m = pose_->pose.pose.position.y;
      pose_->pose.pose.orientation.z = quaternion_from_euler(
        0, 0, theta_rad + theta_rad_dt);
      pose_->pose.pose.position.x = x_m + x_m_dt;
      pose_->pose.pose.position.y = y_m + y_m_dt;

      pose_->twist.twist.linear.x = d_center_m / dt_s;
      pose_->twist.twist.angular.z = phi_rad / dt_s;
    }

    void publish_odometry()
    {
      //rclcpp::Clock clk = rclcpp::Clock();
      //double t = clk.now().seconds();
      // RCLCPP_INFO(this->get_logger(), "Time: %f", t);
      
      odometry_pub_->publish(*pose_);
    }

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_pub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr radps_left_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr radps_right_sub_;
    rclcpp::TimerBase::SharedPtr update_odometry_timer_;
    rclcpp::TimerBase::SharedPtr publish_odometry_timer_;

    float wheel_radius_m_;
    float wheelbase_m_;

    float wheel_radps_left_;
    float wheel_radps_right_;

    nav_msgs::msg::Odometry::SharedPtr pose_;

  public:

    HadabotDriver() : Node("hadabot_driver"), wheel_radius_m_(0.035), wheelbase_m_(0.14), wheel_radps_left_(0.0), wheel_radps_right_(0.0), pose_(std::make_shared<nav_msgs::msg::Odometry>())
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
        "/hadabot/odom", 10);

      update_odometry_timer_ = this->create_wall_timer(
        UPDATE_DT, std::bind(&HadabotDriver::update_odometry, this));

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
