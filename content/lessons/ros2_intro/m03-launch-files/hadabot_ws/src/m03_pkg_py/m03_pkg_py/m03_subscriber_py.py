import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class MySubscriberNode(Node):

    def __init__(self):
        super().__init__('intro_ros2_topic_subscriber_py')
        self.get_logger().info('Python node waiting for a ROS 2 message...')

        self.subscription = self.create_subscription(
            String,
            'my_sub_topic',
            self.listener_callback,
            10)

    def listener_callback(self, msg):
        self.get_logger().info(
            'ROS 2 message received by Python node: "%s"' % msg.data)


def main(args=None):
    rclpy.init(args=args)
    my_subscriber_node = MySubscriberNode()
    rclpy.spin(my_subscriber_node)
    my_subscriber_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
