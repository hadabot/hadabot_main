import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32, Int32MultiArray


class TurnAngleDeg(Node):

    ###########################################################################
    # HADABOT LESSON TO-DO
    # Change the following class constants
    # TICKS_PER_CM - enter the value you measured
    # DRIVE_FOR_N_CM - cm to drive the left wheel fwd, right wheel backwards,
    #                  effectively turning the Turtle in place for some angle
    ###########################################################################
    TICKS_PER_CM = 40.35
    DRIVE_FOR_N_CM = 10.72

    # Wheel power (shouldn't be slower than 0.7 or else the motor won't spin)
    OPT_WHEEL_POWER = 1.0

    def __init__(self):
        super().__init__('drive_straight_for_n_cm')
        self.wheel_power_pub_left_ = self.create_publisher(
            Float32, '/hadabot/wheel_power_left', 10)
        self.wheel_power_pub_right_ = self.create_publisher(
            Float32, '/hadabot/wheel_power_right', 10)

        self.encoders_sub_ = self.create_subscription(
            Int32MultiArray, '/hadabot/wheel_encoders',
            self.encoder_callback, 10)
        self.encoder_count_left_ = 0
        self.encoder_count_right_ = 0

        self.get_logger().info(
            'Driving Left Wheel Forward, Right Wheel Backwards')
        self.publish_wheel_power(
            self.OPT_WHEEL_POWER, -1.0 * self.OPT_WHEEL_POWER)
        self.future = rclpy.task.Future()

    def encoder_callback(self, msg):
        self.encoder_count_left_ += msg.data[0]
        self.encoder_count_right_ += msg.data[1]

        n_ticks = float(self.TICKS_PER_CM) * float(self.DRIVE_FOR_N_CM)
        if self.encoder_count_left_ >= n_ticks or \
                self.encoder_count_right_ <= (n_ticks * -1.0):
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
        self.get_logger().info('Stopping Motors')

        left_cm = float(self.encoder_count_left_) / float(self.TICKS_PER_CM)
        right_cm = float(self.encoder_count_right_) / float(self.TICKS_PER_CM)
        self.get_logger().info(
            f'Left wheel traveled {left_cm} cm')
        self.get_logger().info(
            f'Right wheel traveled {right_cm} cm')
        self.future.set_result(True)


def main(args=None):
    rclpy.init(args=args)
    hadabot_node = TurnAngleDeg()
    rclpy.spin_until_future_complete(hadabot_node, hadabot_node.future)

    # Destroy the node explicitly
    hadabot_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
