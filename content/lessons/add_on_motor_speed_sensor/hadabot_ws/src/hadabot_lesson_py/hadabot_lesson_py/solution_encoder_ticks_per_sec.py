import rclpy
from rclpy.node import Node
from threading import Lock

from std_msgs.msg import Int32MultiArray, Float32


class EncoderTicksPerSec(Node):
    def __init__(self):
        super().__init__('solution_encoder_ticks_per_sec_py')
        self.create_subscription(
            Int32MultiArray,
            '/hadabot/wheel_encoders', self.encoder_tick_cb, 10)

        self.tick_per_sec_pub_ = self.create_publisher(
            Float32, '/hadabot_lesson/en_ticks_per_sec_left', 10)
        timer_period = 2.0  # seconds
        self.timer_ = self.create_timer(
            timer_period, self.publish_left_ticks_per_sec_cb)

        self.en_tick_mutex_ = Lock()
        self.cur_en_ticks_left_ = 0
        self.cur_en_ticks_right_ = 0
        self.last_publish_time_ = self.get_clock().now()

    def encoder_tick_cb(self, msg):
        left_ticks = msg.data[0]
        right_ticks = msg.data[1]

        self.en_tick_mutex_.acquire()
        self.cur_en_ticks_left_ += left_ticks
        self.cur_en_ticks_right_ += right_ticks
        self.en_tick_mutex_.release()

    def publish_left_ticks_per_sec_cb(self):
        now = self.get_clock().now()

        self.en_tick_mutex_.acquire()
        ticks_left = self.cur_en_ticks_left_
        self.cur_en_ticks_left_ = 0
        self.en_tick_mutex_.release()

        since_last_publish_sec = (
            now - self.last_publish_time_).nanoseconds / 1000000000.0
        ticks_per_sec = ticks_left / since_last_publish_sec

        msg = Float32()
        msg.data = ticks_per_sec
        self.tick_per_sec_pub_.publish(msg)

        self.last_publish_time_ = now


def main(args=None):
    rclpy.init(args=args)

    encoder_ticks_per_sec = EncoderTicksPerSec()

    rclpy.spin(encoder_ticks_per_sec)

    encoder_ticks_per_sec.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
