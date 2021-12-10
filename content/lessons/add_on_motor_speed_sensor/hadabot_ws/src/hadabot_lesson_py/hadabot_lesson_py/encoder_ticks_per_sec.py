import rclpy
from rclpy.node import Node

from std_msgs.msg import String


class EncoderTicksPerSec(Node):

    def __init__(self):
        super().__init__('encoder_ticks_per_sec_py')
        self.subscription = self.create_subscription(
            String,
            'topic',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)


def main(args=None):
    rclpy.init(args=args)

    encoder_ticks_per_sec = EncoderTicksPerSec()

    rclpy.spin(encoder_ticks_per_sec)

    encoder_ticks_per_sec.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
