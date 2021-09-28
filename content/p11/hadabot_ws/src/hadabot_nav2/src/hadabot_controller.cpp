#include <cstdio>
#include <chrono>
#include <memory>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"

using namespace std::chrono_literals;
using std::placeholders::_1;

#define PUBLISH_DT 100ms

#define RUN_MODE_UPDATE_DT 1000ms
typedef enum
{
  rm_go = 1,
  rm_stop01 = 2,
} RunMode;

#define PI 3.14159265

#define TICKS_PER_REVOLUTION 1080.0
#define RADIANS_PER_TICK ((2.0 * PI) / TICKS_PER_REVOLUTION)

class HadabotController : public rclcpp::Node
{
private:
  RunMode run_mode_;
  rclcpp::TimerBase::SharedPtr update_run_mode_timer_;

  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr wheel_power_right_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr wheel_power_left_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_pub_;

  rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr encoders_sub_;
  rclcpp::Time prev_odom_update_time_;

  rclcpp::TimerBase::SharedPtr update_odometry_timer_;
  rclcpp::TimerBase::SharedPtr publish_odometry_timer_;

  float wheel_radius_m_;
  float wheelbase_m_;

  int wheel_encoder_left_;
  int wheel_encoder_right_;

  float wheel_power_left_;
  float wheel_power_right_;
  float wheel_power_min_;

  nav_msgs::msg::Odometry::SharedPtr pose_;

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twist_sub_;

  /***************************************************************************/
  void wheel_encoders_cb(const std_msgs::msg::Int32MultiArray::SharedPtr msg)
  {
    this->wheel_encoder_left_ = msg->data[0];
    this->wheel_encoder_right_ = msg->data[1];
    this->update_odometry();

    // Psuedo-PID controller for wheels - increment min power if wheels stalled
    // but should be spinning. Else decrement min power if wheels are spinning
    // If all stopped, then do nothing to min power.
    float min_power_step = 0.05;
    bool wheel_not_turning =
        (wheel_power_left_ != 0.0 && this->wheel_encoder_left_ == 0.0) ||
        (wheel_power_right_ != 0.0 && this->wheel_encoder_right_ == 0.0);
    bool full_stopped =
        (wheel_power_left_ == 0.0 && this->wheel_encoder_left_ == 0.0) &&
        (wheel_power_right_ == 0.0 && this->wheel_encoder_right_ == 0.0);
    float adj_power_factor =
        wheel_not_turning ? 1.0 : (full_stopped ? 0.0 : -1.0);

    auto prev_wheel_power_min = this->wheel_power_min_;
    this->wheel_power_min_ += (min_power_step * adj_power_factor);

    this->wheel_power_min_ =
        std::min(std::max(double(this->wheel_power_min_), 0.0), 0.7);

    // RCLCPP_INFO(this->get_logger(), "Min power %f", this->wheel_power_min_);

    // New min power, so update wheel speed
    if (prev_wheel_power_min != this->wheel_power_min_)
    {
      send_update_wheel_powers(
          this->wheel_power_right_, this->wheel_power_left_);
    }
  }

  /***************************************************************************/
  void update_odometry()
  {
    // Get the time delta since last update
    auto time_now = this->now();
    double dt_s = (time_now - this->prev_odom_update_time_).seconds();

    // Update previous update time
    this->prev_odom_update_time_ = time_now;

    // Compute distance traveled for each wheel
    float d_left_m = wheel_encoder_left_ * RADIANS_PER_TICK * wheel_radius_m_;
    float d_right_m = wheel_encoder_right_ * RADIANS_PER_TICK * wheel_radius_m_;

    auto d_center_m = (d_right_m + d_left_m) / 2.0;
    auto phi_rad = (d_right_m - d_left_m) / wheelbase_m_;

    // Get current yaw orientation from quaternion
    tf2::Quaternion q_current_orientation(
        pose_->pose.pose.orientation.x,
        pose_->pose.pose.orientation.y,
        pose_->pose.pose.orientation.z,
        pose_->pose.pose.orientation.w);
    tf2::Matrix3x3 m(q_current_orientation);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    auto theta_rad = yaw;

    // How much did we rotate?
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
  void send_update_wheel_powers(float v_r, float v_l)
  {
    std_msgs::msg::Float32 pow_r;
    std_msgs::msg::Float32 pow_l;

    // Normalize for range btw min_power and 1.0
    // Do not modify origin v_r v_l passed in
    auto pow_tmp_r = v_r * (1.0 - this->wheel_power_min_);
    pow_r.data = (pow_tmp_r > 0.0) ? (pow_tmp_r + this->wheel_power_min_) : ((pow_tmp_r < 0.0) ? (pow_tmp_r - this->wheel_power_min_) : 0.0);
    auto pow_tmp_l = v_l * (1.0 - this->wheel_power_min_);
    pow_l.data = (pow_tmp_l > 0.0) ? (pow_tmp_l + this->wheel_power_min_) : ((pow_tmp_l < 0.0) ? (pow_tmp_l - this->wheel_power_min_) : 0.0);

    wheel_power_right_pub_->publish(pow_r);
    wheel_power_left_pub_->publish(pow_l);

    // Update the wheel power with original values (not min power adjusted ones)
    this->wheel_power_left_ = v_l;
    this->wheel_power_right_ = v_r;
  }

  /***************************************************************************/
  void twist_cb(const geometry_msgs::msg::Twist::SharedPtr twist_msg)
  {
    if (this->run_mode_ != rm_go)
    {
      return;
    }

    float v = twist_msg->linear.x;
    float w = twist_msg->angular.z;

    // v_l and v_r in radians per second
    auto v_r = ((2.0 * v) + (w * wheelbase_m_)) / (2 * wheel_radius_m_);
    auto v_l = ((2.0 * v) - (w * wheelbase_m_)) / (2 * wheel_radius_m_);

    // RCLCPP_INFO(this->get_logger(), "Left %f - Right %f", v_l, v_r);

    // Max radians per second for Hadabot wheels ~= X radians
    // Normalize btw -1 and 1.0
    float max_rot_radians = 7.0;
    v_r = ((max_rot_radians + v_r) / max_rot_radians) - 1.0;
    v_l = ((max_rot_radians + v_l) / max_rot_radians) - 1.0;

    v_r = std::min(std::max(v_r, -1.0), 1.0);
    v_l = std::min(std::max(v_l, -1.0), 1.0);

    this->send_update_wheel_powers(v_r, v_l);
  }

  /***************************************************************************/
  void update_run_mode_cb()
  {
    std_msgs::msg::Float32 pow_zero;
    switch (this->run_mode_)
    {
    case rm_go:
      pow_zero.data = 0.0;
      wheel_power_left_pub_->publish(pow_zero);
      wheel_power_right_pub_->publish(pow_zero);
      this->run_mode_ = rm_stop01;
      break;
    case rm_stop01:
      this->run_mode_ = rm_go;
      break;
    default:
      break;
    }
  }

public:
  HadabotController() : Node("hadabot_controller"), wheel_radius_m_(0.035),
                        wheelbase_m_(0.14),
                        wheel_encoder_left_(0), wheel_encoder_right_(0),
                        wheel_power_left_(0.0), wheel_power_right_(0.0),
                        wheel_power_min_(0.0)
  {
    RCLCPP_INFO(this->get_logger(), "Starting Hadabot Controller");

    // Init run mode
    run_mode_ = rm_go;
    //update_run_mode_timer_ = this->create_wall_timer(
    //    RUN_MODE_UPDATE_DT,
    //    std::bind(&HadabotController::update_run_mode_cb, this));

    // Initialize pose
    pose_ = std::make_shared<nav_msgs::msg::Odometry>();
    tf2::Quaternion q;
    q.setEuler(0, 0, 0);
    pose_->pose.pose.orientation.x = q.getX();
    pose_->pose.pose.orientation.y = q.getY();
    pose_->pose.pose.orientation.z = q.getZ();
    pose_->pose.pose.orientation.w = q.getW();
    pose_->pose.pose.position.x = 0.0;
    pose_->pose.pose.position.y = 0.0;
    pose_->pose.pose.position.z = 0.0;

    // Current time
    this->prev_odom_update_time_ = this->now();

    odometry_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(
        "/hadabot/odom", 10);

    twist_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "/hadabot/cmd_vel", 10,
        std::bind(&HadabotController::twist_cb, this, _1));

    encoders_sub_ = this->create_subscription<std_msgs::msg::Int32MultiArray>(
        "/hadabot/wheel_encoders", 10,
        std::bind(&HadabotController::wheel_encoders_cb, this, _1));

    wheel_power_left_pub_ = this->create_publisher<std_msgs::msg::Float32>(
        "/hadabot/wheel_power_left", 10);
    wheel_power_right_pub_ = this->create_publisher<std_msgs::msg::Float32>(
        "/hadabot/wheel_power_right", 10);

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
