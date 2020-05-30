#include <cstdio>
#include <chrono>
#include <memory>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "tf2/LinearMath/Quaternion.h"

using namespace std::chrono_literals;
using std::placeholders::_1;

typedef enum {
  LEFT,
  RIGHT
} HBSide;

#define UPDATE_DT 100ms
#define PUBLISH_DT 500ms

#define PI 3.14159265

class HadabotController : public rclcpp::Node
{
private:

  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr wheel_power_right_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr wheel_power_left_pub_;
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

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twist_sub_;

  /***************************************************************************/
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

  /***************************************************************************/
  void wheel_radps_left_cb(const std_msgs::msg::Float32::SharedPtr msg)
  {
    this->wheel_radps(msg, HBSide::LEFT);
  }

  /***************************************************************************/
  void wheel_radps_right_cb(const std_msgs::msg::Float32::SharedPtr msg)
  {
    this->wheel_radps(msg, HBSide::RIGHT);
  }

  /***************************************************************************/
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
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, theta_rad + theta_rad_dt);
    pose_->pose.pose.orientation.x = q.getX();
    pose_->pose.pose.orientation.y = q.getY();
    pose_->pose.pose.orientation.z = q.getZ(); 
    pose_->pose.pose.orientation.w = q.getW(); 
    pose_->pose.pose.position.x = x_m + x_m_dt;
    pose_->pose.pose.position.y = y_m + y_m_dt;
    pose_->twist.twist.linear.x = d_center_m / dt_s;
    pose_->twist.twist.angular.z = phi_rad / dt_s;
  }

  /***************************************************************************/
  void publish_odometry()
  {
    odometry_pub_->publish(*pose_);
  }

  /***************************************************************************/
  void twist_cb(const geometry_msgs::msg::Twist::SharedPtr twist_msg)
  {
    float v = twist_msg->linear.x;
    float w = twist_msg->angular.z;

    // Implement your unicycle to differential drive converstion below
    auto v_r = v * w * 0.0;
    auto v_l = v * w * 0.0;

    std_msgs::msg::Float32 pow_r;
    std_msgs::msg::Float32 pow_l;
    pow_r.data = std::min(std::max(v_r, -1.0), 1.0);
    pow_l.data = std::min(std::max(v_l, -1.0), 1.0);

    wheel_power_left_pub_->publish(pow_l);
    wheel_power_right_pub_->publish(pow_r);
  }

public:
  HadabotController() : Node("hadabot_controller"), wheel_radius_m_(0.035), wheelbase_m_(0.14), wheel_radps_left_(0.0), wheel_radps_right_(0.0), pose_(std::make_shared<nav_msgs::msg::Odometry>())
  {
    twist_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "/hadabot/cmd_vel", 10,
        std::bind(&HadabotController::twist_cb, this, _1));

    radps_left_sub_ = this->create_subscription<std_msgs::msg::Float32>(
        "/hadabot/wheel_radps_left", 10,
        std::bind(&HadabotController::wheel_radps_left_cb, this, _1)
      );

    radps_right_sub_ = this->create_subscription<std_msgs::msg::Float32>(
      "/hadabot/wheel_radps_right", 10,
      std::bind(&HadabotController::wheel_radps_right_cb, this, _1)
    );

    odometry_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(
      "/hadabot/odom", 10);

    wheel_power_left_pub_ = this->create_publisher<std_msgs::msg::Float32>(
      "/hadabot/wheel_power_left", 10);
    wheel_power_right_pub_ = this->create_publisher<std_msgs::msg::Float32>(
      "/hadabot/wheel_power_right", 10);

    update_odometry_timer_ = this->create_wall_timer(
      UPDATE_DT, std::bind(&HadabotController::update_odometry, this));

    publish_odometry_timer_ = this->create_wall_timer(
      PUBLISH_DT, std::bind(&HadabotController::publish_odometry, this));
  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<HadabotController>());
  rclcpp::shutdown();
  return 0;
}
