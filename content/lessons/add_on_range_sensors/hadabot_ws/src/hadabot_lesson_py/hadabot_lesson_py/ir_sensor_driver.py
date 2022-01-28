import rclpy
from rclpy.node import Node

from std_msgs.msg import Int32MultiArray
from sensor_msgs.msg import Range


class IrSensorDriver(Node):

    def __init__(self):
        super().__init__('ir_sensor_driver_py')
        self.create_subscription(
            Int32MultiArray,
            '/hadabot/ir_sensors_12bit_adc', self.ir_sensor_adc_cb, 10)

        self.range_pubs_ = []

    def ir_sensor_adc_cb(self, msg):
        ir_sensor_orientation_labels = msg.layout.dim[0].label.split(
            "_")

        if len(self.range_pubs_) == 0:
            self.init_range_publishers(
                ir_sensor_orientation_labels)

        for idx, range_pub in enumerate(
                self.range_pubs_):
            frame_id = "range_sensor_" + \
                ir_sensor_orientation_labels[idx]
            self.publish_range(
                range_pub, msg.data[idx],
                frame_id)

    def init_range_publishers(
            self, ir_sensor_orientation_labels):

        for orientation_label in ir_sensor_orientation_labels:
            # To publish out range in meters
            range_pub = self.create_publisher(
                Range, '/hadabot/range_sensor/' +
                orientation_label, 10)
            self.range_pubs_.append(range_pub)

    def publish_range(
            self, range_pub, range_12bit_adc,
            frame_id):
        # Field of view characteristic of the ir range sensor
        fov_half_radians = (3.0/360.0) * 6.28

        ######################################################################
        # HADABOT LESSON TO-DO
        #
        # Implement the conversion from range_12bit_adc, which is an integer
        # between 0 and 4095 to a distance in meters
        ######################################################################
        dist = 0.0

        range_msg = Range()
        range_msg.header.stamp.sec = 0
        range_msg.header.stamp.nanosec = 0
        range_msg.header.frame_id = frame_id
        range_msg.radiation_type = Range.INFRARED
        range_msg.field_of_view = fov_half_radians

        # IR Range sensor we use has a min range of 10cm and max range of 80cm
        range_msg.min_range = 0.1
        range_msg.max_range = 0.8

        range_msg.range = dist
        range_pub.publish(range_msg)


def main(args=None):
    rclpy.init(args=args)

    ir_sensor_driver = IrSensorDriver()

    rclpy.spin(ir_sensor_driver)

    ir_sensor_driver.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
