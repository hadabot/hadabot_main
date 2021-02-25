import rclpy
from rclpy.node import Node


class IntroNode(Node):

    def __init__(self):
        super().__init__('intro_ros2_node_py')
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        self.get_logger().info(
            'Hello from the Hadabot ROS 2 intro Python node: "%d"' % self.i)
        self.i += 1


def main(args=None):
    rclpy.init(args=args)

    intro_node = IntroNode()

    rclpy.spin(intro_node)

    intro_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
