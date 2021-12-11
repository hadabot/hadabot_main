import rclpy
from rclpy.node import Node
from threading import Lock

from std_msgs.msg import Int32MultiArray, Float32


class EncoderTicksPerSec(Node):
    def __init__(self):
        super().__init__('encoder_ticks_per_sec_py')

        ########################################################################
        # HADABOT LESSON TO-DO
        # Create a subscriber to the '/hadabot/wheel_encoders' Int32MultiArray
        #   topic, which calls the self.encoder_tick_cb callback
        #
        # Create a publisher that publishes the left encoder's
        #   ticks per second to a '/hadabot_lesson/en_ticks_per_sec_left'
        #   topic using a Float32 message.
        #
        # Create a timer that calls the self.publish_left_ticks_per_sec_cb
        #   every 2 seconds to publish out on to the
        #   /hadabot_lesson/en_ticks_per_sec_left
        #
        # Other useful ROS client library API's we used
        # - self.get_clock().now() - to get the current time
        # - the threading.Lock() mutex to prevent the publisher and
        #   subscriber from modifying shared variables
        #
        # References to help you:
        # - https://docs.ros.org/en/foxy/Tutorials/Writing-A-Simple-Py-Publisher-And-Subscriber.html
        # - https://docs.ros2.org/foxy/api/rclpy/index.html
        # - https://github.com/ros2/rclpy
        ########################################################################

    def encoder_tick_cb(self, msg):
        pass

    def publish_left_ticks_per_sec_cb(self):
        pass


def main(args=None):
    rclpy.init(args=args)

    encoder_ticks_per_sec = EncoderTicksPerSec()

    rclpy.spin(encoder_ticks_per_sec)

    encoder_ticks_per_sec.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
