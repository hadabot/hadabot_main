from uhadabot.uroslibpy import Ros, Topic, Message
from boot import CONFIG
from machine import Pin, PWM
import math
import time
import logging


logger = logging.getLogger(__name__)

BUILTIN_LED = 2


M_LEFT_PWM = 12
M_RIGHT_PWM = 14
M_LEFT_FR = 13
M_RIGHT_FR = 15

M_LEFT_PWM_PIN = None
M_RIGHT_PWM_PIN = None
M_LEFT_FR_PIN = None
M_RIGHT_FR_PIN = None

EN_LEFT = 16
EN_RIGHT = 17


###############################################################################
class Encoder:
    TICKS_PER_REVOLUTION = 40.0
    DEBOUNCE_US = 8000

    def __init__(self, name):
        self.name = name

        # Used by IRQ handler, need semaphore?
        self.en_count = 0
        self.en_count_us = time.ticks_us()

        # Used for debouncing code
        self.en_debounce_us = 0
        # END Used by IRQ handler

        self.ros = None
        self.ros_pub = None

        # Used for when we publish out the tick count
        self.last_pub_us = 0
        self.last_pub_en_count = 0
        self.last_pub_en_count_us = self.en_count_us
        self.last_pub_en_count_diff = 0

    def irq_handler(self, pin):
        t_us = time.ticks_us()
        if time.ticks_diff(t_us, self.en_debounce_us) > self.DEBOUNCE_US:
            self.en_count += 1
            self.en_count_us = t_us

        # We assume real signal is less than debounce interval
        # so always update the last bounce time
        self.en_debounce_us = t_us

    def init_ros(self, ros):
        self.ros = ros
        self.ros_pub = Topic(
            ros, "hadabot/wheel_radps_{}".format(self.name), "std_msgs/Float32")

    def run_once(self):
        cur_us = time.ticks_us()

        # Do we need to disable IRQ to avoid race conditions??
        en_count = self.en_count
        en_count_us = self.en_count_us

        # If the rotation changed, then publish more frequently. Even more
        # frequently if stopped from a moving velocity
        nochange_pub_freq_us = 2000000

        # The number of ticks between current tick count versus what we
        # published last time
        en_count_diff = en_count - self.last_pub_en_count

        # If stopped from a moving velocity then publish that asap
        changed_pub_freq_us = 100000 if en_count_diff == 0 else 200000

        # Was the last published en_count delta same as what we published last
        # time?
        last_pub_same_as_current = self.last_pub_en_count_diff == en_count_diff

        # If last published en_count delta is different from current delta,
        # and enough time has elapsed, publish out
        need_to_pub_change = (
            (time.ticks_diff(
                cur_us, self.last_pub_us) > (changed_pub_freq_us)) and
            (last_pub_same_as_current is False))

        # If last published en_count delta is equal to current delta,
        # and a longer time has elapsed, then publish out as well
        need_to_pub_unchanged = time.ticks_diff(
            cur_us, self.last_pub_us) > (nochange_pub_freq_us)

        # Do we need to publish out?
        if need_to_pub_unchanged or need_to_pub_change:
            # Publish out radians per sec
            time_delta_us = time.ticks_diff(
                en_count_us, self.last_pub_en_count_us)
            if self.last_pub_en_count_diff == 0:
                # If the wheels were not spinning, the last encoder update
                # could have happened aeons ago, so use last published
                # time for time delta to get more accurate rad per sec.
                # Only significant if need_to_pub_change is true
                time_delta_us = time.ticks_diff(en_count_us, self.last_pub_us)
            rps = 0.0

            # Could be less than zero because our last encoder update could
            # have happened before the last published update of unchanged
            # state.
            if time_delta_us > 0:
                # Radians per second
                radians = 2 * math.pi * ((
                    en_count - self.last_pub_en_count) /
                    self.TICKS_PER_REVOLUTION)
                rps = radians * 1000000.0 / float(time_delta_us)
            self.ros_pub.publish(Message({"data": rps}))

            # Update values published
            self.last_pub_en_count = en_count
            self.last_pub_en_count_us = en_count_us
            self.last_pub_us = cur_us
            self.last_pub_en_count_diff = en_count_diff


###############################################################################
def turn_wheel(wheel_power_f32, pwm_pin, fr_pin):
    factor = max(min(wheel_power_f32, 1.0), -1.0)

    if factor >= 0:
        # FR pin lo to go forward
        fr_pin.off()
        pwm_pin.duty(int(1023 * factor))
    else:
        # FR pin hi and 'reverse' pwm to go backwards
        fr_pin.on()
        pwm_pin.duty(1023 - int(1023 * (-1*factor)))


###############################################################################
def right_wheel_cb(wheel_power):
    turn_wheel(wheel_power["data"], M_RIGHT_PWM_PIN, M_RIGHT_FR_PIN)


###############################################################################
def left_wheel_cb(wheel_power):
    turn_wheel(wheel_power["data"], M_LEFT_PWM_PIN, M_LEFT_FR_PIN)


###############################################################################
def blink_led(ntimes, end_off):
    builtin_led = Pin(BUILTIN_LED, Pin.OUT)

    # Start off
    builtin_led.off()
    time.sleep(0.5)

    for x in range(0, ntimes):
        builtin_led.off()
        time.sleep(0.25)
        builtin_led.on()
        time.sleep(0.5)

    if end_off:
        builtin_led.off()


###############################################################################
def twist_cmd_cb(twist_msg):
    # Blink and leave LED on
    blink_led(1, end_off=False)


###############################################################################
def setup_pins(en_left, en_right):
    global M_LEFT_PWM_PIN
    global M_RIGHT_PWM_PIN
    global M_LEFT_FR_PIN
    global M_RIGHT_FR_PIN

    # Motor pings
    M_LEFT_PWM_PIN = PWM(Pin(M_LEFT_PWM))
    M_RIGHT_PWM_PIN = PWM(Pin(M_RIGHT_PWM))

    M_LEFT_FR_PIN = Pin(M_LEFT_FR, Pin.OUT)
    M_RIGHT_FR_PIN = Pin(M_RIGHT_FR, Pin.OUT)

    M_LEFT_PWM_PIN.duty(0)
    M_RIGHT_PWM_PIN.duty(0)
    M_LEFT_FR_PIN.off()
    M_RIGHT_FR_PIN.off()

    # Encoder interrupts
    en_pin_left = Pin(EN_LEFT, Pin.IN)
    en_pin_right = Pin(EN_RIGHT, Pin.IN)

    en_pin_left.irq(trigger=(Pin.IRQ_FALLING | Pin.IRQ_RISING),
                    handler=en_left.irq_handler)
    en_pin_right.irq(trigger=(Pin.IRQ_FALLING | Pin.IRQ_RISING),
                     handler=en_right.irq_handler)


###############################################################################
def main(argv):
    ros = None

    en_left = Encoder("left")
    en_right = Encoder("right")

    setup_pins(en_left, en_right)

    try:
        hadabot_ip_address = CONFIG["network"]["hadabot_ip_address"]
        boot_button = Pin(0, Pin.IN)
        builtin_led = Pin(BUILTIN_LED, Pin.OUT)

        ros = Ros(CONFIG["ros2_web_bridge_ip_addr"])

        # Publish out log info
        log_info = Topic(ros, "hadabot/log/info", "std_msgs/String")

        # Subscribe to twist topics
        twist_cmd = Topic(ros, "hadabot/cmd_vel", "geometry_msgs/Twist")
        twist_cmd.subscribe(twist_cmd_cb)

        # Subscribe to wheel turn callbacks
        rw_sub_topic = Topic(
            ros, "hadabot/wheel_power_right", "std_msgs/Float32")
        rw_sub_topic.subscribe(right_wheel_cb)
        lw_sub_topic = Topic(
            ros, "hadabot/wheel_power_left", "std_msgs/Float32")
        lw_sub_topic.subscribe(left_wheel_cb)

        # Publish out encoder messages
        en_left.init_ros(ros)
        en_right.init_ros(ros)

        # Loop forever
        last_hb_ms = time.ticks_ms()
        while(boot_button.value() == 1):
            ros.run_once()

            cur_ms = time.ticks_ms()

            # Encoder management includes publishing out if necessary
            en_left.run_once()
            en_right.run_once()

            # Publish heartbeat message
            if time.ticks_diff(cur_ms, last_hb_ms) > 5000:
                # logger.info("Encoder left - {}".format(en_count_left))
                # logger.info("Encoder right - {}".format(en_count_right))
                log_info.publish(Message(
                    "Hadabot heartbeat - IP {}".format(hadabot_ip_address)))
                last_hb_ms = cur_ms

            time.sleep(0.01)

        logger.info("Boot button pushed... exiting")

    except KeyboardInterrupt:
        logger.info("Keyboard control-c hit... stopping")
    except Exception as ex:
        logger.error("Some other exception hit {}".format(str(ex)))
        if ros is not None:
            ros.terminate()

        # Blink 5 times
        blink_led(5, end_off=True)

        raise ex

    # Shutdown
    ros.terminate()
    builtin_led.off()


main(None)
