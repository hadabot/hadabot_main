import rclpy
from rclpy.node import Node

from std_msgs.msg import Int32, Float32, Int32MultiArray


class DriveCirclePattern(Node):

    ###########################################################################
    # HADABOT LESSON TO-DO
    # Change the following class constants
    # TICKS_PER_CM - enter the value you measured
    ###########################################################################
    TICKS_PER_CM = 1.0

    # Wheel power (shouldn't be slower than 0.7 or else the motor won't spin)
    OPT_WHEEL_POWER = 1.0

    def __init__(self):
        super().__init__('solution_drive_circle_pattern')
        self.wheel_power_pub_left_ = self.create_publisher(
            Float32, '/hadabot/wheel_power_left', 10)
        self.wheel_power_pub_right_ = self.create_publisher(
            Float32, '/hadabot/wheel_power_right', 10)

        self.encoders_sub_ = self.create_subscription(
            Int32MultiArray, '/hadabot/wheel_encoders',
            self.encoder_callback, 10)
        self.encoder_count_left_ = 0
        self.encoder_count_right_ = 0

        self.drive_for_n_cm_ = 0

        # HADABOT LESSON TO-DO
        # Implement the drive_circle_n_cm_sub_ subscription
        # for an Int32 message that
        # will be received over the topic named
        # '/drive_circle_n_cm', upon which the
        # drive_circle_n_cm_callback(...) will be called
        self.drive_circle_n_cm_sub_ = None

        self.get_logger().info('Waiting for /drive_circle_n_cm message...')
        self.publish_wheel_power(0.0, 0.0)

        self.future_ = rclpy.task.Future()

    def drive_circle_n_cm_callback(self, msg):
        self.encoder_count_left_ = 0
        self.encoder_count_right_ = 0
        self.drive_for_n_cm = msg.data
        self.publish_wheel_power(self.OPT_WHEEL_POWER, 0.0)

    def encoder_callback(self, msg):
        self.encoder_count_left_ += msg.data[0]
        self.encoder_count_right_ += msg.data[1]

        if self.drive_for_n_cm_ > 0:
            n_ticks_threshold = float(self.TICKS_PER_CM) * self.drive_for_n_cm_
            if self.encoder_count_left_ >= n_ticks_threshold:
                self.stop_motor()

    def publish_wheel_power(self, power_left_f32, power_right_f32):
        msg_left = Float32()
        msg_left.data = power_left_f32
        self.wheel_power_pub_left_.publish(msg_left)
        msg_right = Float32()
        msg_right.data = power_right_f32
        self.wheel_power_pub_right_.publish(msg_right)

    def stop_motor(self):
        self.publish_wheel_power(0.0, 0.0)
        self.get_logger().info('Stopped')
        self.future.set_result(True)


def main(args=None):
    rclpy.init(args=args)
    hadabot_node = DriveCirclePattern()
    rclpy.spin_until_future_complete(hadabot_node, hadabot_node.future_)

    # Destroy the node explicitly
    hadabot_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
