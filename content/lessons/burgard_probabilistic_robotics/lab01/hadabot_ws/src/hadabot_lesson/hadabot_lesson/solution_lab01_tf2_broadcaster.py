import numpy as np
import rclpy
from rclpy.node import Node
import tf_transformations
from tf2_ros import TransformBroadcaster
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped


class Lab01TF2Broadcaster(Node):

    def __init__(self):
        super().__init__('solution_lab01_tf2_broadcaster')

        static_tf_pub = StaticTransformBroadcaster(self)

        static_transform_stamped = TransformStamped()
        static_transform_stamped.header.stamp = self.get_clock().now().to_msg()
        static_transform_stamped.header.frame_id = 'robot'
        static_transform_stamped.child_frame_id = 'laser'
        static_transform_stamped.transform.translation.x = 0.2
        static_transform_stamped.transform.translation.y = 0.0
        static_transform_stamped.transform.translation.z = 0.0
        quat = tf_transformations.quaternion_from_euler(0.0, 0.0, np.pi)
        static_transform_stamped.transform.rotation.x = quat[0]
        static_transform_stamped.transform.rotation.y = quat[1]
        static_transform_stamped.transform.rotation.z = quat[2]
        static_transform_stamped.transform.rotation.w = quat[3]
        static_tf_pub.sendTransform(static_transform_stamped)

        # Broadcast world-robot frame periodically
        self.tf_pub_ = TransformBroadcaster(self)
        timer_period = 0.25  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        t = TransformStamped()

        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'world'
        t.child_frame_id = 'robot'

        t.transform.translation.x = 1.0
        t.transform.translation.y = 0.5
        t.transform.translation.z = 0.0

        q = tf_transformations.quaternion_from_euler(0, 0, np.pi * 0.25)
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        # Send the transformation
        self.tf_pub_.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)

    lab01_node = Lab01TF2Broadcaster()

    rclpy.spin(lab01_node)

    lab01_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
