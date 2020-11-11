// Example tf2 python tutorials:
//   https://github.com/ros2/geometry2/tree/ros2/examples_tf2_py

// #include <ros/ros.h>
#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>

using std::placeholders::_1;

class HadabotTF2Publisher : public rclcpp::Node
{
public:
    HadabotTF2Publisher()
        : Node("hadabot_tf2_publisher")
    {
        tf_broadcaster_ =
            std::make_unique<tf2_ros::TransformBroadcaster>(*this);
        subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10,
            std::bind(&HadabotTF2Publisher::odom_callback, this, _1));
    }

private:
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) const
    {
        rclcpp::Time now;
        // RCLCPP_INFO(this->get_logger(), "Pose: '%f'", msg->x);

        geometry_msgs::msg::TransformStamped odom_tf;
        geometry_msgs::msg::TransformStamped base_link_tf;

        base_link_tf.transform.translation.x = msg->pose.pose.position.x;
        base_link_tf.transform.translation.y = msg->pose.pose.position.y;
        base_link_tf.transform.translation.z = 0.0;
        base_link_tf.transform.rotation = msg->pose.pose.orientation;

        base_link_tf.header.frame_id = "odom";
        base_link_tf.child_frame_id = "base_link";
        base_link_tf.header.stamp = now;
        tf_broadcaster_->sendTransform(base_link_tf);

        odom_tf.transform.translation.x = 0.0;
        odom_tf.transform.translation.y = 0.0;
        odom_tf.transform.translation.z = 0.0;
        tf2::Quaternion q;
        q.setRPY(0, 0, 0);
        odom_tf.transform.rotation.x = q.x();
        odom_tf.transform.rotation.y = q.y();
        odom_tf.transform.rotation.z = q.z();
        odom_tf.transform.rotation.w = q.w();

        odom_tf.header.frame_id = "map";
        odom_tf.child_frame_id = "odom";
        odom_tf.header.stamp = now;
        tf_broadcaster_->sendTransform(odom_tf);
    }
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<HadabotTF2Publisher>());
    rclcpp::shutdown();
    return 0;
}