import numpy as np
import rclpy
from rclpy.node import Node
import tf_transformations
from tf2_ros import TransformBroadcaster
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from sensor_msgs.msg import LaserScan


class Lab01RangeSensorDriver(Node):

    def __init__(self):
        super().__init__('lab01_range_sensor_driver')

        # Load range sensor dataimport math
        self.scan = np.loadtxt('../laserscan.dat').tolist()

        # Publish out range sensor data periodically
        self.laser_pub_ = self.create_publisher(
            LaserScan, '/lab01/laser_scan', 10)
        timer_period = 0.25  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        ls_msg = LaserScan()
        ls_msg.header.stamp = self.get_clock().now().to_msg()
        ls_msg.header.frame_id = 'laser'
        ls_msg.angle_min = -1.0 * (np.pi * 0.5)
        ls_msg.angle_max = 1.0 * (np.pi * 0.5)
        ls_msg.angle_increment = (
            (ls_msg.angle_max - ls_msg.angle_min)/float(len(self.scan)-1))
        ls_msg.time_increment = 0.0
        ls_msg.scan_time = 0.1
        ls_msg.range_min = 0.0
        ls_msg.range_max = 1000000.0
        ls_msg.ranges = self.scan
        ls_msg.intensities = []
        self.laser_pub_.publish(ls_msg)


def main(args=None):
    rclpy.init(args=args)

    lab01_node = Lab01RangeSensorDriver()

    rclpy.spin(lab01_node)

    lab01_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
