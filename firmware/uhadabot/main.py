from uhadabot.uroslibpy import Ros, Topic, Message
from boot import CONFIG
from machine import Pin, PWM
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

DEBOUNCE_MS = 2000
debounce_left = 0
debounce_right = 0
EN_LEFT = 16
EN_RIGHT = 17
en_count_left = 0
en_count_right = 0


###############################################################################
def encoder_handler_left(pin):
    global en_count_left
    global debounce_left

    # N microseconds since last interrupt
    m = time.ticks_us()
    if (m - debounce_left) > DEBOUNCE_MS:
        en_count_left += 1

    # We assume real signal is less than debounce ms
    # so always update the last bounce time
    debounce_left = m


###############################################################################
def encoder_handler_right(pin):
    global en_count_right
    global debounce_right

    # N microseconds since last interrupt
    m = time.ticks_us()
    if (m - debounce_right) > DEBOUNCE_MS:
        en_count_right += 1

    # We assume real signal is less than debounce ms
    # so always update the last bounce time
    debounce_right = m


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
def setup_pins():
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
                    handler=encoder_handler_left)
    en_pin_right.irq(trigger=(Pin.IRQ_FALLING | Pin.IRQ_RISING),
                     handler=encoder_handler_right)


###############################################################################


def main(argv):
    ros = None

    setup_pins()

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

        # Loop forever
        cnt = 0
        while(boot_button.value() == 1):
            ros.run_once()

            if (cnt % 1000) == 0:
                # logger.info("Encoder left - {}".format(en_count_left))
                # logger.info("Encoder right - {}".format(en_count_right))
                log_info.publish(Message(
                    "Hadabot heartbeat - IP {}".format(hadabot_ip_address)))

            cnt += 1
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
