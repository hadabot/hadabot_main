import network
import logging
import time

logger = logging.getLogger(__name__)


def main():
    max_tries = 15
    sta_if = network.WLAN(network.STA_IF)
    ip = sta_if.ifconfig()[0]
    idx = 0
    while ip == "0.0.0.0" and idx < max_tries:
        logger.info("Still waiting to get an IP address from router...")
        time.sleep(1)
        ip = sta_if.ifconfig()[0]
        idx += 1

    if ip == "0.0.0.0":
        logger.info("Could not get IP address of the Hadabot control board")
    else:
        print("")
        print("The Websocket URL for the Hadabot control board: ")
        print("  ws://{}:8266/".format(ip))
        print("")


main()
