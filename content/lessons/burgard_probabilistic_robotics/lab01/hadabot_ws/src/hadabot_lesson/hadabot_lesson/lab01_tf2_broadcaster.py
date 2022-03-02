import numpy as np
import rclpy
from rclpy.node import Node
import tf_transformations
from tf2_ros import TransformBroadcaster
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped


class Lab01TF2Broadcaster(Node):

    def __init__(self):
        super().__init__('lab01_tf2_broadcaster')

        ######################################################################
        # HADABOT LESSON TO-DO
        #
        # Using the solution_lab01_tf2_broadcaster.py as reference, implement
        # the code to publish the static transform frames between the
        # 'robot' frame and 'laser' frame
        ######################################################################

        # Broadcast world-robot frame periodically
        self.tf_pub_ = TransformBroadcaster(self)
        timer_period = 0.25  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        t = TransformStamped()

        ######################################################################
        # HADABOT LESSON TO-DO
        #
        # Using the solution_lab01_tf2_broadcaster.py as reference, implement
        # the code to publish the transform frames between the
        # 'world' frame and 'robot' frame
        ######################################################################


def main(args=None):
    rclpy.init(args=args)

    lab01_node = Lab01TF2Broadcaster()

    rclpy.spin(lab01_node)

    lab01_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
