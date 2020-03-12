# Copyright 2020 - Hadabot.com

# This file is executed on every boot (including wake-boot from deepsleep)
# import esp
# esp.osdebug(None)
import webrepl
import json
import os
import time

CONFIG_FILE = "hb.json"
CONFIG = None


###############################################################################
def do_install_requirements():
    try:
        import logging  # noqa
    except Exception:
        import upip
        upip.install("micropython-logging")


###############################################################################
def do_start_network():
    import machine
    import network
    sta_if = network.WLAN(network.STA_IF)

    if CONFIG["network"]["ssid"].startswith("XX") and \
       CONFIG["network"]["ssid"].endswith("XX"):
        raise Exception(
            "Please edit your network configuration in the ./{} file, "
            "then re-upload the file to the ESP32 board.".format(
                CONFIG_FILE))

    led_pin = machine.Pin(2, machine.Pin.OUT)
    if not sta_if.isconnected():
        print('Connecting to network {}...'.format(CONFIG["network"]["ssid"]))
        sta_if.active(True)
        sta_if.connect(CONFIG["network"]["ssid"],
                       CONFIG["network"]["password"])
        while not sta_if.isconnected():
            print('Waiting for network to connect...')
            led_pin.on()
            time.sleep(0.5)
            led_pin.off()
            time.sleep(0.5)
            pass
    print('Network connected. Config: ', sta_if.ifconfig())
    led_pin.on()


###############################################################################
def do_read_config():
    global CONFIG
    global CONFIG_FILE

    if CONFIG_FILE in os.listdir():
        f = open("/{}".format(CONFIG_FILE), "r")
        r = f.read()
        c = ""
        while r != "":
            c += r
            r = f.read()
        CONFIG = json.loads(c)
        f.close()
    else:
        raise Exception("Could not find a {} config file".format(CONFIG_FILE))


###############################################################################
def do_setup():
    import logging

    logger = logging.getLogger(__name__)
    logger.info("Welcome to Hadabot - www.hadabot.com")

    webrepl.start()


###############################################################################
# Initial boot calls
do_read_config()
do_start_network()
do_install_requirements()
do_setup()
