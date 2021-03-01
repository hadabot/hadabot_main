import rclpy
from rclpy.node import Node


class MyROSNode(Node):

    def __init__(self):
        super().__init__('intro_ros2_node_py')

        # Add timer callback code below
        pass

    def timer_callback(self):
        self.get_logger().info(
            'Hello from the Hadabot ROS 2 intro Python node: "%d"' % self.i)
        self.i += 1


def main(args=None):
    rclpy.init(args=args)

    my_ros_node = MyROSNode()

    rclpy.spin(my_ros_node)

    my_ros_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
