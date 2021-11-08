import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32, Int32MultiArray


class DriveStraightForNSecs(Node):

    ############################################################################
    # HADABOT LESSON TO-DO
    # Change this class constant to change the time to stop
    ############################################################################
    STOP_AFTER_N_SECONDS = 2

    def __init__(self):
        super().__init__('drive_straight_for_n_secs')
        self.wheel_power_pub_left_ = self.create_publisher(
            Float32, '/hadabot/wheel_power_left', 10)
        self.wheel_power_pub_right_ = self.create_publisher(
            Float32, '/hadabot/wheel_power_right', 10)

        self.encoders_sub_ = self.create_subscription(
            Int32MultiArray, '/hadabot/wheel_encoders',
            self.encoder_callback, 10)
        self.encoder_count_left_ = 0
        self.encoder_count_right_ = 0

        timer_period = self.STOP_AFTER_N_SECONDS  # seconds
        self.timer = self.create_timer(timer_period, self.stop_motor_callback)
        self.get_logger().info('Driving Motor Forward')
        self.publish_wheel_power(1.0, 1.0)
        self.future = rclpy.task.Future()

    def encoder_callback(self, msg):
        self.encoder_count_left_ += msg.data[0]
        self.encoder_count_right_ += msg.data[1]

    def publish_wheel_power(self, power_left_f32, power_right_f32):
        msg_left = Float32()
        msg_left.data = power_left_f32
        self.wheel_power_pub_left_.publish(msg_left)
        msg_right = Float32()
        msg_right.data = power_right_f32
        self.wheel_power_pub_left_.publish(msg_right)

    def stop_motor_callback(self):
        self.timer.cancel()
        self.publish_wheel_power(0.0, 0.0)
        self.get_logger().info('Stopping Motors')
        self.get_logger().info(
            f'Left encoder tick count after {self.STOP_AFTER_N_SECONDS} '
            f'seconds: {self.encoder_count_left_}')
        self.get_logger().info(
            f'Right encoder tick count after {self.STOP_AFTER_N_SECONDS} '
            f'seconds: {self.encoder_count_right_}')
        self.future.set_result(True)


def main(args=None):
    rclpy.init(args=args)
    hadabot_node = DriveStraightForNSecs()
    rclpy.spin_until_future_complete(hadabot_node, hadabot_node.future)

    # Destroy the node explicitly
    hadabot_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
