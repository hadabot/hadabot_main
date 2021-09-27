from uhadabot.uroslibpy import Ros, Topic, Message
from boot import CONFIG, PIN_CONFIG
import ssd1306
from machine import Pin, PWM, I2C, Timer, ADC
import time
import logging


logger = logging.getLogger(__name__)

hadabot = None


###############################################################################
class ssd1306_stub:
    def __init__(self):
        pass

    def fill(*args, **kwargs):
        pass

    def text(*args, **kwargs):
        pass

    def show(*args, **kwargs):
        pass


###############################################################################
class RangeSensorSet:

    # 12-bit ADC: 0 to 4095 = 3.6v max
    # 490 (0.43v) to 2740 (2.41v) - 150 (0.13v) intervals - 16 slots
    dist_table_m = [
        0.805, 0.589, 0.460, 0.375, 0.315, 0.271, 0.237, 0.210,
        0.188, 0.171, 0.156, 0.143, 0.132, 0.123, 0.114, 0.107]

    def __init__(self, ros):
        self.ros = ros
        self.r_sensors = []
        for rs_pc in PIN_CONFIG["range_sensors"]:
            rs_adc = ADC(Pin(rs_pc["pin"]))
            rs_adc.atten(ADC.ATTN_11DB)
            rs_adc.width(ADC.WIDTH_12BIT)  # 12bits

            rs_topic = "hadabot/range_sensor/" + rs_pc["label"]
            rs_frame_id = "range_sensor_" + rs_pc["label"]
            self.r_sensors.append({
                "label": rs_pc["label"],
                "frame_id": rs_frame_id,
                "adc_pin": rs_adc,
                "ros_pub": Topic(ros, rs_topic, "sensor_msgs/Range"),
                "fov_half_radians": (3.0/360.0) * 6.28
            })

    def publish_range_m(self):
        for rs in self.r_sensors:
            val_12bit = rs["adc_pin"].read()
            val = max(min(val_12bit, 2739), 490)
            fidx = (val - 490)/150.0
            idx_start = int(fidx)
            frac = fidx - idx_start
            dist = ((self.dist_table_m[idx_start] * (1.0 - frac)) +
                    (self.dist_table_m[idx_start+1] * frac))

            rs["ros_pub"].publish(Message({
                "header": {
                    "stamp": {
                        "sec": 0, "nanosec": 0
                    },
                    "frame_id": rs["frame_id"]
                },
                "radiation_type": 1,
                "field_of_view": rs["fov_half_radians"],
                "min_range": 0.1,
                "max_range": 0.81,
                "range": dist,
            }))

###############################################################################


class Encoder:

    def __init__(self, name, pin):
        self.name = name

        self.en_pin = pin
        self.count = 0
        self.prev_pin_val = pin.value()

        # Encoder interrupts
        self.en_pin.irq(trigger=Pin.IRQ_RISING, handler=self.en_cb)

    def en_cb(self, pin):
        self.count += 1

    def get_reset_count(self):
        cnt = self.count
        self.count = 0
        return cnt


###############################################################################
class EncoderSet:
    def __init__(self, ros, name_pin_tuple_list):

        # Initial encoder ticks message
        dim_label = ""
        self.ros_encoders_message_json = {
            "layout": {
                "dim": [{
                    "label": dim_label,
                    "size": len(name_pin_tuple_list),
                    "stride": len(name_pin_tuple_list)
                }],
                "data_offset": 0,
            },
            "data": [0] * len(name_pin_tuple_list)
        }

        # List of encoders
        self.encoder_list = []
        for name_pin in name_pin_tuple_list:
            # Create encoder object
            self.encoder_list.append(Encoder(name_pin[0], name_pin[1]))
            dim_label += name_pin[0] + "_"

        # Update multi array dim label
        self.ros_encoders_message_json[
            "layout"]["dim"][0]["label"] = dim_label.strip("_")

        self.ros = ros
        self.ros_pub = Topic(
            ros, "hadabot/wheel_encoders", "std_msgs/Int32MultiArray")

    def publish_encoders(self, encoder_ticks_list):
        for idx, ticks in enumerate(encoder_ticks_list):
            self.ros_encoders_message_json["data"][idx] = ticks

        # logger.info("Encoder publish {}".format(self.name))
        self.ros_pub.publish(Message(self.ros_encoders_message_json))

    def get_count(self, channel_idx):
        return self.encoder_list[channel_idx].get_reset_count()


###############################################################################
class Controller:
    TICKS_PER_REVOLUTION = 1080  # 12 ppr (1:90) --- 135 3ppr (1:45)

    ###########################################################################
    def __init__(self, ros):

        # Motor pins
        self.pwm_pin_left = PWM(Pin(PIN_CONFIG["left"]["motor"]["pwm"]))
        self.pwm_pin_right = PWM(Pin(PIN_CONFIG["right"]["motor"]["pwm"]))

        self.fr_pin_left = Pin(PIN_CONFIG["left"]["motor"]["fr"], Pin.OUT)
        self.fr_pin_right = Pin(PIN_CONFIG["right"]["motor"]["fr"], Pin.OUT)

        self.pwm_pin_left.duty(0)
        self.pwm_pin_right.duty(0)
        self.fr_pin_left.off()
        self.fr_pin_right.off()

        # Motor direction (fwd == 1.0, reverse == -1.0, stop == 0.0)
        self.fr_left = 0.0
        self.fr_right = 0.0

        # Encoder
        en_pin_left = Pin(PIN_CONFIG["left"]["encoder"]["a"], Pin.IN)
        en_pin_right = Pin(PIN_CONFIG["right"]["encoder"]["a"], Pin.IN)
        self.en_set = EncoderSet(
            ros, [("left", en_pin_left), ("right", en_pin_right)])

        # Range sensors
        self.rs_set = RangeSensorSet(ros)
        self.last_rs_pub_ms = time.ticks_ms()

    ###########################################################################
    def turn_wheel(self, wheel_power_f32, pwm_pin, fr_pin, prev_direction):
        # factor = max(min(wheel_power_f32, 1.0), -1.0)
        factor = max(min(wheel_power_f32, 1.0), 0.0)  # Only fwd

        # The Hadabot wheels actually don't turn well below a threshold,
        # so let's normalize between -1.0 and thresh, thresh to 1.0
        if factor != 0.0:
            factor = factor * 0.3
            factor = factor + 0.7 if factor > 0 else factor
            factor = factor - 0.7 if factor < 0 else factor
            factor = max(factor, 0.85)  # only care about fwd

        # Send command
        self._send_motor_signal(factor, pwm_pin, fr_pin)

    ###########################################################################
    def _send_motor_signal(self, factor, pwm_pin, fr_pin):
        if factor >= 0:
            # FR pin lo to go forward
            fr_pin.off()
            pwm_pin.duty(int(1023 * factor))
        else:
            # FR pin hi and 'reverse' pwm to go backwards
            fr_pin.on()
            pwm_pin.duty(1023 - int(1023 * (-1*factor)))

    ###########################################################################
    def right_wheel_cb(self, wheel_power):
        self.turn_wheel(
            wheel_power["data"], self.pwm_pin_right, self.fr_pin_right,
            self.fr_right)
        self.fr_right = -1.0 if wheel_power["data"] < 0.0 else 0.0
        self.fr_right = 1.0 if wheel_power["data"] > 0.0 else self.fr_right

    ###########################################################################
    def left_wheel_cb(self, wheel_power):
        self.turn_wheel(
            wheel_power["data"], self.pwm_pin_left, self.fr_pin_left,
            self.fr_left)
        self.fr_left = -1.0 if wheel_power["data"] < 0.0 else 0.0
        self.fr_left = 1.0 if wheel_power["data"] > 0.0 else self.fr_left

    ###########################################################################
    def run_once(self):
        cur_ms = time.ticks_ms()
        # state = machine.disable_irq()
        count_left = self.en_set.get_count(0)
        count_right = self.en_set.get_count(1)
        # machine.enable_irq(state)

        # Publish radps
        self.en_set.publish_encoders([count_left, count_right])

        if time.ticks_diff(cur_ms, self.last_rs_pub_ms) > 500:
            # Publish range data
            self.rs_set.publish_range_m()
            self.last_rs_pub_ms = cur_ms

        if False:
            logger.info(
                "Encoder count {} - {}".format(
                    self.en_left.name, self.en_left.count))
            logger.info(
                "Encoder count {} - {}".format(
                    self.en_right.name, self.en_right.count))


###############################################################################
class Hadabot:

    ###########################################################################
    def __init__(self):
        self.ros = None
        self.spin_timer = None

        try:
            # Start sensors first since it takes some time to power up
            p_sensor_display_power = Pin(
                PIN_CONFIG["power"]["sensors"], Pin.OUT)
            p_sensor_display_power.on()

            # Setup ROS
            self.hadabot_ip_address = CONFIG["network"]["hadabot_ip_address"]
            self.builtin_led = Pin(PIN_CONFIG["led"]["status"], Pin.OUT)
            self.ros = Ros(CONFIG["ros2_web_bridge_ip_addr"])

            # Set up OLED
            self.oled = None
            try:
                i2c = I2C(scl=Pin(PIN_CONFIG["i2c"]["scl"]),
                          sda=Pin(PIN_CONFIG["i2c"]["sda"]))
                oled_width = 128
                oled_height = 64
                self.oled = ssd1306.SSD1306_I2C(oled_width, oled_height, i2c)
            except Exception:
                logger.info("Could not find OLED on I2C")
                self.oled = ssd1306_stub()

            # Write to OLED
            self.oled.fill(0)
            self.oled.text('Hadabot running', 0, 0)
            self.oled.show()

            # Publish out log info
            self.log_info = Topic(self.ros, "hadabot/log/info",
                                  "std_msgs/String")

            # Controller
            self.controller = Controller(self.ros)

            # Subscribe/publish to ping/ping_ack
            self.ping_inertia_stamped_topic = Topic(
                self.ros, "hadabot/ping_inertia_stamped",
                "geometry_msgs/InertiaStamped")
            self.ping_inertia_stamped_topic.subscribe(
                self.ping_inertia_stamped_cb)
            self.ping_inertia_stamped_ack_topic = Topic(
                self.ros, "hadabot/ping_inertia_stamped_ack",
                "geometry_msgs/InertiaStamped")
            self.ping_time_reference_topic = Topic(
                self.ros, "hadabot/ping_time_reference",
                "sensor_msgs/TimeReference")
            self.ping_time_reference_topic.subscribe(
                self.ping_time_reference_cb)
            self.ping_time_reference_ack_topic = Topic(
                self.ros, "hadabot/ping_time_reference_ack",
                "sensor_msgs/TimeReference")

            # Subscribe to blink led
            self.blink_led_sub_topic = Topic(
                self.ros, "hadabot/blink_led", "std_msgs/Int32")
            self.blink_led_sub_topic.subscribe(self.blink_led_cb)

            # Subscribe to wheel turn callbacks
            self.rw_sub_topic = Topic(
                self.ros, "hadabot/wheel_power_right", "std_msgs/Float32")
            self.rw_sub_topic.subscribe(self.controller.right_wheel_cb)
            self.lw_sub_topic = Topic(
                self.ros, "hadabot/wheel_power_left", "std_msgs/Float32")
            self.lw_sub_topic.subscribe(self.controller.left_wheel_cb)

            self.last_hb_ms = time.ticks_ms()
            self.last_controller_ms = time.ticks_ms()

            # Boot button to stop hadabot timer
            self.need_shutdown = False
            self.boot_button = Pin(PIN_CONFIG["button"]["boot"], Pin.IN)
            self.boot_button.irq(trigger=Pin.IRQ_RISING,
                                 handler=self.user_shutdown_cb)

            # Spin timer
            self.spin_timer = Timer(0)
            self.spin_timer.init(period=10, mode=Timer.PERIODIC,
                                 callback=self.spin)

        except Exception as ex:
            logger.error("Init exception hit {}".format(str(ex)))
            self.blink_led(5, end_off=True)
            self.shutdown()
            raise ex

    ###########################################################################
    def user_shutdown_cb(self, pin):
        logger.info("User shutdown invoked")
        self.shutdown()

    ###########################################################################
    def ping_time_reference_cb(self, msg):
        self.ping_time_reference_ack_topic.publish(Message(msg))
        self.last_hb_ms = time.ticks_ms()

    ###########################################################################
    def ping_inertia_stamped_cb(self, msg):
        self.ping_inertia_stamped_ack_topic.publish(Message(msg))
        self.last_hb_ms = time.ticks_ms()

    ###########################################################################
    def blink_led_cb(self, msg):
        ntimes = int(msg["data"])
        self.blink_led(ntimes, end_off=False)

    def blink_led(self, ntimes, end_off):
        # Start off
        self.builtin_led.off()
        time.sleep(0.5)

        for x in range(0, ntimes):
            self.builtin_led.off()
            time.sleep(0.25)
            self.builtin_led.on()
            time.sleep(0.5)

        if end_off:
            self.builtin_led.off()

    ###########################################################################
    def shutdown(self):
        # Shutdown
        self.need_shutdown = True

        self.oled.fill(0)
        self.oled.text('Hadabot stopped', 0, 0)
        self.oled.show()

        if self.ros is not None:
            try:
                self.ros.terminate()
            except Exception:
                pass
            self.ros = None

        self.builtin_led.off()

    ###########################################################################
    def spin(self, tim):
        try:
            if self.need_shutdown:
                raise Exception("Shut down requested")

            self.ros.run_once()

            cur_ms = time.ticks_ms()

            # Run the controller
            if time.ticks_diff(cur_ms, self.last_controller_ms) > 100:
                self.controller.run_once()
                self.last_controller_ms = cur_ms

            # Publish heartbeat message
            if time.ticks_diff(cur_ms, self.last_hb_ms) > 5000:
                # logger.info("Encoder left - {}".format(en_count_left))
                # logger.info("Encoder right - {}".format(en_count_right))
                # self.log_info.publish(Message(
                #    "Hadabot heartbeat - IP {}".format(
                #        self.hadabot_ip_address)))
                self.log_info.publish(Message(str(cur_ms)))
                self.last_hb_ms = cur_ms

        except Exception as ex:
            logger.error("Spin exception hit {}".format(str(ex)))
            tim.deinit()
            # self.blink_led(3, end_off=True)

            # If this was not a user triggered shutdown then reset
            if self.need_shutdown is False:
                logger.error(
                    "Soft resetting doesn't quite work so just shutdown")
                # machine.soft_reset()
                self.shutdown()


###############################################################################
def main(argv):
    global hadabot
    hadabot = Hadabot()


main(None)
