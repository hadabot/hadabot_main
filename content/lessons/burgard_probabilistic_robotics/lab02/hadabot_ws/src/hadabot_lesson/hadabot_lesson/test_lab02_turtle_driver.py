import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32


class Lab02TurtleDriver(Node):

    def __init__(self):
        super().__init__('test_lab02_turtle_driver')

        self.wheel_power_pub_left_ = self.create_publisher(
            Float32, '/hadabot/wheel_power_left', 10)
        self.wheel_power_pub_right_ = self.create_publisher(
            Float32, '/hadabot/wheel_power_right', 10)

        pow_full = Float32()
        pow_full.data = 1.0
        pow_zero = Float32()
        pow_zero.data = 0.0

        time.sleep(1.0)

        # Move forward for 1 second
        self.wheel_power_pub_left_.publish(pow_full)
        self.wheel_power_pub_right_.publish(pow_full)
        self.get_logger().info('Publishing wheel power to move forward')

        time.sleep(1.0)

        # Turn right for 1 second
        self.wheel_power_pub_left_.publish(pow_full)
        self.wheel_power_pub_right_.publish(pow_zero)
        self.get_logger().info('Publishing wheel power to turn right')

        time.sleep(1.0)

        # Stop
        self.wheel_power_pub_left_.publish(pow_zero)
        self.wheel_power_pub_right_.publish(pow_zero)
        self.get_logger().info('Publishing wheel power to stop')


def main(args=None):
    rclpy.init(args=args)

    lab02_node = Lab02TurtleDriver()

    # rclpy.spin(lab02_node)

    lab02_node.destroy_node()
    rclpy.shutdown()
