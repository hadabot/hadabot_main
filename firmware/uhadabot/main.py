from uhadabot.uroslibpy import Ros, Topic, Message
from boot import CONFIG
from machine import Pin
import time
import logging


logger = logging.getLogger(__name__)

BUILTIN_LED = 2


###############################################################################
def main(argv):
    boot_button = Pin(0, Pin.IN)
    builtin_led = Pin(BUILTIN_LED, Pin.OUT)

    ros = Ros(CONFIG["ros2_web_bridge_ip_addr"])
    hb_out = Topic(ros, "/hadabot_output", "std_msgs/String")
    cnt = 0

    # Loop forever
    try:
        while(boot_button.value() == 1):
            ros.run_once()

            if (cnt % 1000) == 0:
                hb_out.publish(Message(
                    "ROSbots' alive {}".format(cnt)))

            cnt += 1
            time.sleep(0.01)

        logger.info("Boot button pushed... exiting")

    except KeyboardInterrupt:
        logger.info("Keyboard control-c hit... stopping")
    except Exception as ex:
        logger.error("Some other exception hit {}".format(str(ex)))
        ros.terminate()
        builtin_led.off()
        raise ex

    # Shutdown
    ros.terminate()
    builtin_led.off()


main(None)
