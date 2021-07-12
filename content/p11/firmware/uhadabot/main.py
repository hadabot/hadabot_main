from uhadabot.uroslibpy import Ros, Topic, Message
from boot import CONFIG, PIN_CONFIG
import ssd1306
from machine import Pin, PWM, I2C, Timer
import math
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
class Encoder:

    def __init__(self, ros, name, pin):
        self.name = name

        self.en_pin = pin
        self.count = 0
        self.prev_pin_val = pin.value()

        # Encoder interrupts
        self.en_pin.irq(trigger=Pin.IRQ_RISING, handler=self.en_cb)

        self.ros = ros
        self.ros_pub = Topic(
            ros, "hadabot/wheel_radps_{}".format(self.name),
            "std_msgs/Float32")

    def en_cb(self, pin):
        self.count += 1

    def publish_radps(self, radps):
        # logger.info("Encoder publish {}".format(self.name))
        self.ros_pub.publish(Message({"data": radps}))


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

        self.en_left = Encoder(ros, "left", en_pin_left)
        self.en_right = Encoder(ros, "right", en_pin_right)

        # Encoder samples
        self.prev_count_left = 0
        self.prev_count_right = 0
        self.prev_ms = time.ticks_ms()

    ###########################################################################
    def irq_poll_en(self, timer):
        for en in [self.en_right, self.en_left]:
            val = en.en_pin.value()
            if (val ^ en.prev_pin_val) == 1:
                en.count += 1
                en.prev_pin_val = val

    ###########################################################################
    def turn_wheel(self, wheel_power_f32, pwm_pin, fr_pin, prev_direction):
        factor = max(min(wheel_power_f32, 1.0), -1.0)

        # The Hadabot wheels actually don't turn well below 0.5, so let's
        # normalize between -1.0 and -0.5, 0.5 to 1.0
        if False:
            factor = factor * 0.5
            factor = factor + 0.5 if factor > 0 else factor
            factor = factor - 0.5 if factor < 0 else factor

        # Overdrive motors to get them spinning, then back off to the
        # speed we desire (hack for not having a PID controller)
        dir_change_sleep_ms = 50
        if prev_direction * factor == 0:
            if factor > 0:
                self._send_motor_signal(0.8, pwm_pin, fr_pin)
                time.sleep_ms(dir_change_sleep_ms)
            elif factor < 0:
                self._send_motor_signal(-0.8, pwm_pin, fr_pin)
                time.sleep_ms(dir_change_sleep_ms)
        elif prev_direction * factor < 0:
            # Changing direction - stop motor first
            self._send_motor_signal(0.0, pwm_pin, fr_pin)
            time.sleep_ms(dir_change_sleep_ms)

            # Then overdrive in the opposite direction from
            # the previous direction
            self._send_motor_signal(-0.8 * prev_direction, pwm_pin, fr_pin)
            time.sleep_ms(dir_change_sleep_ms)

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
        # state = machine.disable_irq()
        count_left = self.en_left.count
        count_right = self.en_right.count
        # machine.enable_irq(state)

        cur_ms = time.ticks_ms()
        time_delta_ms = time.ticks_diff(cur_ms, self.prev_ms)
        per_second = 1000.0 / float(time_delta_ms)

        # Left encoder
        dcount_left = count_left - self.prev_count_left
        radians = 2 * math.pi * (dcount_left / self.TICKS_PER_REVOLUTION)
        radps_left = radians * per_second
        self.en_left.publish_radps(radps_left * self.fr_left)

        # Right encoder
        dcount_right = count_right - self.prev_count_right
        radians = 2 * math.pi * (dcount_right / self.TICKS_PER_REVOLUTION)
        radps_right = radians * per_second
        self.en_right.publish_radps(radps_right * self.fr_right)

        # Update previous sample
        self.prev_ms = cur_ms
        self.prev_count_left = count_left
        self.prev_count_right = count_right

        if False:
            logger.info(
                "Encoder count {} - {}".format(
                    self.en_left.name, self.en_left.count))
            logger.info(
                "Encoder count {} - {}".format(
                    self.en_right.name, self.en_right.count))
            if count_left >= 120:
                self.left_wheel_cb({"data": 0.01})
            if count_right >= 120:
                self.right_wheel_cb({"data": 0.01})


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
            self.ping_string_topic = Topic(
                self.ros, "hadabot/ping_string", "std_msgs/String")
            self.ping_string_topic.subscribe(self.ping_string_cb)
            self.ping_string_ack_topic = Topic(self.ros,
                                               "hadabot/ping_string_ack",
                                               "std_msgs/String")
            self.ping_f32_topic = Topic(
                self.ros, "hadabot/ping_f32", "std_msgs/Float32")
            self.ping_f32_topic.subscribe(self.ping_f32_cb)
            self.ping_f32_ack_topic = Topic(self.ros,
                                            "hadabot/ping_f32_ack",
                                            "std_msgs/Float32")

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
    def ping_f32_cb(self, msg):
        msg_data = float(msg["data"])
        self.ping_f32_ack_topic.publish(Message({"data": msg_data}))
        self.last_hb_ms = time.ticks_ms()

    ###########################################################################
    def ping_string_cb(self, msg):
        msg_data = msg["data"]
        self.ping_string_ack_topic.publish(Message(msg_data))
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
            self.ros.terminate()
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
                #self.log_info.publish(Message(
                #    "Hadabot heartbeat - IP {}".format(
                #        self.hadabot_ip_address)))
                self.log_info.publish(Message(str(cur_ms)))
                self.last_hb_ms = cur_ms

        except Exception as ex:
            logger.error("Spin exception hit {}".format(str(ex)))
            tim.deinit()
            self.blink_led(3, end_off=True)
            self.shutdown()


###############################################################################
def main(argv):
    global hadabot
    hadabot = Hadabot()


main(None)
