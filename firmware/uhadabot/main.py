from uhadabot.uroslibpy import Ros, Topic, Message
from boot import CONFIG
from machine import Pin
import time
import logging


logger = logging.getLogger(__name__)

BUILTIN_LED = 2


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
def main(argv):
    ros = None
    try:
        boot_button = Pin(0, Pin.IN)
        builtin_led = Pin(BUILTIN_LED, Pin.OUT)

        ros = Ros(CONFIG["ros2_web_bridge_ip_addr"])
        log_info = Topic(ros, "/hadabot/log/info", "std_msgs/String")

        twist_cmd = Topic(ros, "/hadabot/cmd_vel", "geometry_msgs/Twist")
        twist_cmd.subscribe(twist_cmd_cb)

        cnt = 0

        # Loop forever
        while(boot_button.value() == 1):
            ros.run_once()

            if (cnt % 1000) == 0:
                log_info.publish(Message(
                    "Hadabot heartbeat {}".format(cnt * 0.001)))

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
