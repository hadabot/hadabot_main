import numpy as np
from threading import Lock
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray


def diffdrive(x, y, theta, v_l, v_r, t, wheelbase):
    # straight line
    if (v_l == v_r):
        theta_n = theta
        x_n = x + v_l * t * np.cos(theta)
        y_n = y + v_l * t * np.sin(theta)

    # circular motion
    else:
        # Calculate the radius
        R = wheelbase/2.0 * ((v_l + v_r) / (v_r - v_l))

        # computing center of curvature
        ICC_x = x - R * np.sin(theta)
        ICC_y = y + R * np.cos(theta)

        # compute the angular velocity
        omega = (v_r - v_l) / wheelbase

        # computing angle change
        dtheta = omega * t

        # forward kinematics for differential drive
        x_n = np.cos(dtheta)*(x-ICC_x) - np.sin(dtheta)*(y-ICC_y) + ICC_x
        y_n = np.sin(dtheta)*(x-ICC_x) + np.cos(dtheta)*(y-ICC_y) + ICC_y
        theta_n = theta + dtheta
    return x_n, y_n, theta_n


class Lab02DiffDrive(Node):

    WHEEL_RADIUS_M = 0.035  # 3.5cm

    ###########################################################################
    # HADABOT LESSON TO-DO
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
        super().__init__('solution_lab02_diff_drive')
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

        ts_delta_sec = (
            cur_ts - self.prev_ts_).nanoseconds / 1000000000.0
        self.prev_ts_ = cur_ts

        v_l = (
            (float(encoder_count_left)/float(self.TICKS_PER_2_PI)) *
            (2.0 * np.pi * self.WHEEL_RADIUS_M) * (1.0 / ts_delta_sec))
        v_r = (
            (float(encoder_count_right)/float(self.TICKS_PER_2_PI)) *
            (2.0 * np.pi * self.WHEEL_RADIUS_M) * (1.0 / ts_delta_sec))

        (x_n, y_n, theta_n) = diffdrive(
            self.x_, self.y_, self.theta_,
            v_l, v_r, ts_delta_sec, self.WHEELBASE_M)

        self.en_tick_mutex_.acquire()
        self.x_ = x_n
        self.y_ = y_n
        self.theta_ = theta_n
        self.en_tick_mutex_.release()

    def timer_callback(self):
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
