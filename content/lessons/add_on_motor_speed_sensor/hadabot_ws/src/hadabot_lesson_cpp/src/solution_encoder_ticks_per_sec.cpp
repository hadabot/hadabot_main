#include <memory>
#include <mutex>
#include <chrono>
#include <functional>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"
#include "std_msgs/msg/float32.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

class EncoderTicksPerSec : public rclcpp::Node
{
public:
  EncoderTicksPerSec()
      : Node("solution_encoder_ticks_per_sec"), cur_en_ticks_left_(0), cur_en_ticks_right_(0)
  {
    this->wheel_encoders_sub_ =
        this->create_subscription<std_msgs::msg::Int32MultiArray>(
            "/hadabot/wheel_encoders", 10, std::bind(&EncoderTicksPerSec::encoder_tick_cb, this, _1));

    tick_per_sec_pub_ = this->create_publisher<std_msgs::msg::Float32>(
        "/hadabot_lesson/en_ticks_per_sec_left", 10);
    timer_ = this->create_wall_timer(
        2s, std::bind(
                &EncoderTicksPerSec::publish_left_ticks_per_sec_cb, this));

    last_publish_time_ = this->now();
  }

private:
  void encoder_tick_cb(const std_msgs::msg::Int32MultiArray::SharedPtr msg)
  {
    auto left_ticks = msg->data[0];
    auto right_ticks = msg->data[1];

    this->en_tick_mutex_.lock();
    this->cur_en_ticks_left_ += left_ticks;
    this->cur_en_ticks_right_ += right_ticks;
    this->en_tick_mutex_.unlock();
  }

  void publish_left_ticks_per_sec_cb()
  {
    auto now = this->now();

    this->en_tick_mutex_.lock();
    auto ticks_left = this->cur_en_ticks_left_;
    this->cur_en_ticks_left_ = 0;
    this->en_tick_mutex_.unlock();

    float since_last_publish_sec =
        (now - this->last_publish_time_).nanoseconds() / 1000000000.0;
    float ticks_per_sec = ticks_left / since_last_publish_sec;

    auto msg = std_msgs::msg::Float32();
    msg.data = ticks_per_sec;
    tick_per_sec_pub_->publish(msg);

    this->last_publish_time_ = now;
  }

  rclcpp::Subscription<
      std_msgs::msg::Int32MultiArray>::SharedPtr wheel_encoders_sub_;
  std::mutex en_tick_mutex_;
  int cur_en_ticks_left_;
  int cur_en_ticks_right_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr tick_per_sec_pub_;
  rclcpp::Time last_publish_time_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<EncoderTicksPerSec>());
  rclcpp::shutdown();
  return 0;
}