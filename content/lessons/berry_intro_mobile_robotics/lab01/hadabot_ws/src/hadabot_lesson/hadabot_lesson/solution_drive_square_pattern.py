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
    TICKS_PER_CM = 40.35
    HADABOT_WHEELBASE_CM = 13.5

    # Wheel power (shouldn't be slower than 0.7 or else the motor won't spin)
    OPT_WHEEL_POWER = 1.0

    STATE_DRIVE_STRAIGHT = 1
    STATE_TURN_RIGHT_90_DEG = 2
    STATE_DONE = 3

    SQUARE_PATTERN_N_EDGES = 4

    def __init__(self):
        super().__init__('solution_drive_square_pattern')
        self.wheel_power_pub_left_ = self.create_publisher(
            Float32, '/hadabot/wheel_power_left', 10)
        self.wheel_power_pub_right_ = self.create_publisher(
            Float32, '/hadabot/wheel_power_right', 10)

        self.encoders_sub_ = self.create_subscription(
            Int32MultiArray, '/hadabot/wheel_encoders',
            self.encoder_callback, 10)
        self.encoder_count_left_ = 0
        self.encoder_count_right_ = 0

        self.drive_square_n_edges_drawn_ = 0
        self.drive_square_state_machine_ = self.STATE_DRIVE_STRAIGHT

        self.get_logger().info('Driving Motor Forward Along Left Edge')
        self.drive_straight()

        self.future_ = rclpy.task.Future()

    ###########################################################################
    #
    def drove_for_40cm(self):
        n_ticks_threshold = float(self.TICKS_PER_CM) * 40.0
        if self.encoder_count_left_ >= n_ticks_threshold or \
                self.encoder_count_right_ >= n_ticks_threshold:
            return True
        return False

    def turned_right_90deg(self):
        math_pi = 3.14159
        n_cm_for_90deg = (float(self.HADABOT_WHEELBASE_CM) * math_pi) * 0.25
        n_ticks_threshold = float(self.TICKS_PER_CM) * n_cm_for_90deg
        if self.encoder_count_left_ >= n_ticks_threshold or \
                self.encoder_count_right_ <= (n_ticks_threshold * -1.0):
            return True
        return False

    def encoder_callback(self, msg):
        self.encoder_count_left_ += msg.data[0]
        self.encoder_count_right_ += msg.data[1]

        if self.drive_square_state_machine_ == self.STATE_DRIVE_STRAIGHT:
            transition = self.drove_for_40cm()

            if transition:
                self.drive_square_state_machine_ = self.STATE_TURN_RIGHT_90_DEG
                self.reset_encoder_ticks()
                self.turn_right()
        elif self.drive_square_state_machine_ == self.STATE_TURN_RIGHT_90_DEG:
            transition = self.turned_right_90deg()

            if transition:
                self.drive_square_n_edges_drawn_ += 1

                if self.drive_square_n_edges_drawn_ == \
                        self.SQUARE_PATTERN_N_EDGES:
                    # Already drove 4 edges, just stop
                    self.drive_square_state_machine_ = self.STATE_DONE
                    self.stop_motor()
                else:
                    # Need to draw another edge
                    self.drive_square_state_machine_ = \
                        self.STATE_DRIVE_STRAIGHT
                    self.reset_encoder_ticks()
                    self.drive_straight()
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

    def drive_straight(self):
        self.publish_wheel_power(0.0, 0.0)
        self.get_logger().info('Driving straight...')
        self.publish_wheel_power(
            self.OPT_WHEEL_POWER, self.OPT_WHEEL_POWER)

    def turn_right(self):
        self.publish_wheel_power(0.0, 0.0)
        self.get_logger().info('Turning right...')
        self.publish_wheel_power(
            self.OPT_WHEEL_POWER, -1.0 * self.OPT_WHEEL_POWER)

    def stop_motor(self):
        self.publish_wheel_power(0.0, 0.0)
        self.get_logger().info('Stopped')

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
