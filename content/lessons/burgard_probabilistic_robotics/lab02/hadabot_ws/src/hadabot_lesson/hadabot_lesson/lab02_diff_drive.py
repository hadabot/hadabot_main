import numpy as np
from threading import Lock
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray


def diffdrive(x, y, theta, v_l, v_r, t, wheelbase):
    x_n = 0
    y_n = 0
    theta_n = 0

    ######################################################################
    # HADABOT LESSON TO-DO
    #
    # Part 1
    #
    # Implement the forward kinematics equation for differential drive
    # mobile robots.
    ######################################################################
    return (x_n, y_n, theta_n)


class Lab02DiffDrive(Node):

    WHEEL_RADIUS_M = 0.035  # 3.5cm

    ###########################################################################
    # HADABOT LESSON TO-DO
    #
    # Part 2a
    #
    # Change the following class constants
    # WHEELBASE_M - your measured wheelbase for the chassis specific to your
    #             - Turtle robot (either 0.137 meters / 13.7cm -or- 0.14m)
    #
    # TICKS_PER_2_PI - enter the number of ticks per wheel revolution for
    #                - the encoder motor assembly you have (either 1080 or 270)
    ###########################################################################
    WHEELBASE_M = 0.137  # 13.7cm
    TICKS_PER_2_PI = 1080

    def __init__(self):
        super().__init__('lab02_diff_drive')
        self.x_ = 0.0
        self.y_ = 0.0
        self.theta_ = 0.0

        self.encoders_sub_ = self.create_subscription(
            Int32MultiArray, '/hadabot/wheel_encoders',
            self.encoder_callback, 10)
        self.encoder_count_left_ = 0
        self.encoder_count_right_ = 0
        self.prev_ts_ = self.get_clock().now()

        self.en_tick_mutex_ = Lock()

        timer_period = 0.25  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def encoder_callback(self, msg):
        cur_ts = self.get_clock().now()
        encoder_count_left = msg.data[0]
        encoder_count_right = msg.data[1]

        #######################################################################
        # HADABOT LESSON TO-DO
        #
        # Part 2b
        #
        # Each call back will be made with the encoder ticks counted since
        # the previous callback (ie the ticks are not 'cumulatiuve',
        # if the Turtle has not moved, the encoder count will be zero).
        #
        # Finish the implementation to compute the following:
        # - ts_delta_sec - the drive time
        #   (HINT) Try to store away the previous timestamp various to
        #   compute a time delta
        #
        # - v_l and v_r for the wheel velocities in meters per second
        #   (HINT) Use the WHEEL_RADIUS_M and TICKS_PER_2_PI class
        #   variables
        #######################################################################
        ts_delta_sec = 0
        v_l = 0
        v_r = 0

        # Compute the pose update using the diffdrive kinematics code you
        # implemented
        (x_n, y_n, theta_n) = diffdrive(
            self.x_, self.y_, self.theta_,
            v_l, v_r, ts_delta_sec, self.WHEELBASE_M)

        # Store away the current pose update
        self.en_tick_mutex_.acquire()
        self.x_ = x_n
        self.y_ = y_n
        self.theta_ = theta_n
        self.en_tick_mutex_.release()

    def timer_callback(self):
        # Print out the pose
        self.en_tick_mutex_.acquire()
        self.get_logger().info(
            'Current pose (x, y, theta in radians): '
            f'({self.x_, self.y_, self.theta_})')
        self.en_tick_mutex_.release()


def main(args=None):
    rclpy.init(args=args)

    lab02_node = Lab02DiffDrive()

    rclpy.spin(lab02_node)

    lab02_node.destroy_node()
    rclpy.shutdown()
