#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/subscription_options.hpp"

#include "std_msgs/msg/float32.hpp"
#include "geometry_msgs/msg/inertia_stamped.hpp"
#include "sensor_msgs/msg/time_reference.hpp"

class MinimalSubscriberWithTopicStatistics : public rclcpp::Node
{
public:
    MinimalSubscriberWithTopicStatistics()
        : Node("minimal_subscriber_with_topic_statistics")
    {
        // manually enable topic statistics via options
        auto options = rclcpp::SubscriptionOptions();
        options.topic_stats_options.state = rclcpp::TopicStatisticsState::Enable;

        // configure the collection window and publish period (default 1s)
        options.topic_stats_options.publish_period = std::chrono::seconds(10);

        // configure the topic name (default '/statistics')
        options.topic_stats_options.publish_topic = "/statistics/ping_inertia_stamped";

        auto callback = [this](geometry_msgs::msg::InertiaStamped::SharedPtr msg) {
            this->topic_callback(msg);
        };

        subscription_is_ = this->create_subscription<geometry_msgs::msg::InertiaStamped>(
            "hadabot/ping_inertia_stamped_ack", 10, callback, options);

        // Statistics for time reference
        auto options_tr = rclcpp::SubscriptionOptions();
        options_tr.topic_stats_options.state = rclcpp::TopicStatisticsState::Enable;

        // configure the collection window and publish period (default 1s)
        options_tr.topic_stats_options.publish_period = std::chrono::seconds(10);

        // configure the topic name (default '/statistics')
        options_tr.topic_stats_options.publish_topic = "/statistics/ping_time_reference";

        auto callback_tr = [this](sensor_msgs::msg::TimeReference::SharedPtr msg) {
            this->topic_time_reference_callback(msg);
        };

        subscription_tr_ = this->create_subscription<sensor_msgs::msg::TimeReference>(
            "hadabot/ping_time_reference_ack", 10, callback_tr, options_tr);

        // Statistics for wheel radps
        auto options_wr = rclcpp::SubscriptionOptions();
        options_wr.topic_stats_options.state = rclcpp::TopicStatisticsState::Enable;

        // configure the collection window and publish period (default 1s)
        options_wr.topic_stats_options.publish_period = std::chrono::seconds(10);

        // configure the topic name (default '/statistics')
        options_wr.topic_stats_options.publish_topic = "/statistics/wheel_radps_left";

        auto callback_wr = [this](std_msgs::msg::Float32::SharedPtr msg) {
            this->topic_wheel_radps_callback(msg);
        };

        subscription_wr_ = this->create_subscription<std_msgs::msg::Float32>(
            "hadabot/wheel_radps_left", 10, callback_wr, options_wr);
    }

private:
    void topic_wheel_radps_callback(const std_msgs::msg::Float32::SharedPtr) const
    {
        //RCLCPP_INFO(this->get_logger(), "I heard wheel power");
    }
    void topic_callback(const geometry_msgs::msg::InertiaStamped::SharedPtr msg) const
    {
        RCLCPP_INFO(this->get_logger(), "I heard inertia stamped: '%i'", msg->header.stamp.sec);
    }
    void topic_time_reference_callback(const sensor_msgs::msg::TimeReference::SharedPtr msg) const
    {
        RCLCPP_INFO(this->get_logger(), "I heard time reference: '%i'", msg->header.stamp.sec);
    }
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr subscription_wr_;
    rclcpp::Subscription<geometry_msgs::msg::InertiaStamped>::SharedPtr subscription_is_;
    rclcpp::Subscription<sensor_msgs::msg::TimeReference>::SharedPtr subscription_tr_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MinimalSubscriberWithTopicStatistics>());
    rclcpp::shutdown();
    return 0;
}