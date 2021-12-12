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
      : Node("encoder_ticks_per_sec")
  {
    /**************************************************************************
      HADABOT LESSON TO-DO
      Create a subscriber to the '/hadabot/wheel_encoders' Int32MultiArray
        topic, which calls the this->encoder_tick_cb callback
    
      Create a publisher that publishes the left encoder's
        ticks per second to a '/hadabot_lesson/en_ticks_per_sec_left'
        topic using a Float32 message.
    
      Create a timer that calls the this->publish_left_ticks_per_sec_cb
        every 2 seconds to publish out on to the
        /hadabot_lesson/en_ticks_per_sec_left
    
      Other useful ROS client library API's we used
      - this->now() - to get the current time
      - the std::mutex to prevent the publisher and
        subscriber from modifying shared variables at the same time
    
      References to help you:
      - https://docs.ros.org/en/foxy/Tutorials/Writing-A-Simple-Cpp-Publisher-And-Subscriber.html
      - https://docs.ros2.org/foxy/api/rclcpp/index.html
      - https://github.com/ros2/rclcpp
    *************************************************************************/
  }

private:
  void encoder_tick_cb(const std_msgs::msg::Int32MultiArray::SharedPtr msg)
  {
  }

  void publish_left_ticks_per_sec_cb()
  {
  }
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<EncoderTicksPerSec>());
  rclcpp::shutdown();
  return 0;
}