import numpy as np
import rclpy
from rclpy.node import Node


class Lab02DiffDrive(Node):

    def __init__(self):
        super().__init__('solution_lab02_diff_drive')

        timer_period = 0.25  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        pass


def main(args=None):
    rclpy.init(args=args)

    lab02_node = Lab02DiffDrive()

    rclpy.spin(lab02_node)

    lab01_node.destroy_node()
    rclpy.shutdown()
