import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32, Int32MultiArray


class DriveSquarePattern(Node):

    ###########################################################################
    # HADABOT LESSON TO-DO
    # Change the following class constants
    # TICKS_PER_CM - enter the value you measured
    # TURTLE_WHEELBASE_CM - wheelbase of your Turtle
    ###########################################################################
    TICKS_PER_CM = 1.0
    HADABOT_WHEELBASE_CM = 1.0

    # Wheel power (shouldn't be slower than 0.7 or else the motor won't spin)
    OPT_WHEEL_POWER = 1.0

    STATE_DRIVE_STRAIGHT_LEFT_EDGE = 1
    STATE_TURN_RIGHT_90_DEG_UPPER_LEFT = 2
    STATE_DRIVE_STRAIGHT_TOP_EDGE = 3
    STATE_TURN_RIGHT_90_DEG_UPPER_RIGHT = 4
    STATE_DRIVE_STRAIGHT_RIGHT_EDGE = 5
    STATE_TURN_RIGHT_90_DEG_LOWER_RIGHT = 6
    STATE_DRIVE_STRAIGHT_BOTTOM_EDGE = 7
    STATE_TURN_RIGHT_90_DEG_LOWER_LEFT = 8

    def __init__(self):
        super().__init__('drive_square_pattern')
        self.wheel_power_pub_left_ = self.create_publisher(
            Float32, '/hadabot/wheel_power_left', 10)
        self.wheel_power_pub_right_ = self.create_publisher(
            Float32, '/hadabot/wheel_power_right', 10)

        self.encoders_sub_ = self.create_subscription(
            Int32MultiArray, '/hadabot/wheel_encoders',
            self.encoder_callback, 10)
        self.encoder_count_left_ = 0
        self.encoder_count_right_ = 0

        self.drive_square_state_machine_ = \
            self.STATE_DRIVE_STRAIGHT_LEFT_EDGE
        self.get_logger().info('Driving Motor Forward')
        self.publish_wheel_power(self.OPT_WHEEL_POWER, self.OPT_WHEEL_POWER)
        self.future_ = rclpy.task.Future()

    ###########################################################################
    #
    def encoder_callback(self, msg):
        self.encoder_count_left_ += msg.data[0]
        self.encoder_count_right_ += msg.data[1]

        # State machine
        if self.drive_square_state_machine_ == \
                self.STATE_DRIVE_STRAIGHT_LEFT_EDGE:
            # HADABOT LESSON TO-DO
            # Implement the condition to transition to the next state
            transition = True

            if transition:
                self.drive_square_state_machine = \
                    self.STATE_TURN_RIGHT_90_DEG_UPPER_LEFT
                self.reset_encoder_ticks()
                self.turn_right()
        elif self.drive_square_state_machine == \
                self.STATE_TURN_RIGHT_90_DEG_UPPER_LEFT:
            transition = True  # HADABOT LESSON TO-DO

            if transition:
                self.drive_square_state_machine = \
                    self.STATE_DRIVE_STRAIGHT_TOP_EDGE
                self.reset_encoder_ticks()
                self.drive_straight()
        elif self.drive_square_state_machine == \
                self.STATE_DRIVE_STRAIGHT_TOP_EDGE:
            # HADABOT LESSON TO-DO
            # Implement all the other transition states
            # (ie a lot more elif's!)
            pass
    #
    ###########################################################################

    def reset_encoder_ticks(self):
        self.encoder_count_left_ = 0
        self.encoder_count_right_ = 0

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
        self.future_.set_result(True)


def main(args=None):
    rclpy.init(args=args)
    hadabot_node = DriveSquarePattern()
    rclpy.spin_until_future_complete(hadabot_node, hadabot_node.future_)

    # Destroy the node explicitly
    hadabot_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
