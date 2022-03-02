import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class Lab01RangeSensorDriver(Node):

    def __init__(self):
        super().__init__('solution_lab01_range_sensor_driver')

        # Load range sensor dataimport math
        self.scan = np.loadtxt('../laserscan.dat').tolist()

        # Publish out range sensor data periodically
        self.laser_pub_ = self.create_publisher(
            LaserScan, '/lab01/laser_scan', 10)
        timer_period = 0.25  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        ls_msg = LaserScan()

        ######################################################################
        # HADABOT LESSON TO-DO
        #
        # Using the solution_lab01_range_sensor_driver.py as reference,
        # implement the code to publish the laser scan data as a LaserScan
        # ROS message
        ######################################################################

        self.laser_pub_.publish(ls_msg)


def main(args=None):
    rclpy.init(args=args)

    lab01_node = Lab01RangeSensorDriver()

    rclpy.spin(lab01_node)

    lab01_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
