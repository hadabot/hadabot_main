#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/inertia_stamped.hpp"
#include "sensor_msgs/msg/time_reference.hpp"

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

class MinimalPublisher : public rclcpp::Node
{
public:
    MinimalPublisher()
        : Node("time_reference_pub"), count_(0)
    {
        publisher_ = this->create_publisher<std_msgs::msg::String>("topic_time_reference", 10);
        publisher_tr_ = this->create_publisher<sensor_msgs::msg::TimeReference>("hadabot/ping_time_reference", 10);
        timer_ = this->create_wall_timer(
            500ms, std::bind(&MinimalPublisher::timer_callback, this));
    }

private:
    void timer_callback()
    {
        auto message = std_msgs::msg::String();
        message.data = "Hello, world! " + std::to_string(count_++);

        auto message_tr = sensor_msgs::msg::TimeReference();
        message_tr.header.stamp = this->now();
        publisher_tr_->publish(message_tr);

        RCLCPP_INFO(this->get_logger(), "Time reference publishing: '%s'", message.data.c_str());
        publisher_->publish(message);
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::Publisher<sensor_msgs::msg::TimeReference>::SharedPtr publisher_tr_;
    size_t count_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MinimalPublisher>());
    rclcpp::shutdown();
    return 0;
}