# Copyright 2020 - Hadabot.com

# This file is executed on every boot (including wake-boot from deepsleep)
# import esp
# esp.osdebug(None)
import machine
import webrepl
import json
import os
import time

CONFIG_FILE = "hb.json"
CONFIG = None

PIN_CONFIG = {
    "button": {"boot": 0},
    "led": {"status": 2},  # 5 - p4r
    "power": {"sensors": 27},
    "i2c": {"scl": 22, "sda": 21},
    "left": {
        "motor": {"pwm": 18, "fr": 19},
        "encoder": {
            "a": 15, "b": 39,
            "ppr": 1080  # 1:90 * 12 -- 1:45 * 3
        }
    },
    "right": {
        "motor": {"pwm": 13, "fr": 14},
        "encoder": {
            "a": 4, "b": 25,
            "ppr": 1080
        }
    },
    "range_sensors": [
        {
            "label": "front", "pin": 32,
            "ori_rpy_deg": [0, 0, 0],
            "pos_xyz_m": [0.07, 0, 0]
        },
        {
            "label": "front_left", "pin": 33,
            "ori_rpy_deg": [0, 0, 25],
            "pos_xyz_m": [0.05, 0.05, 0]
        },
        {
            "label": "front_right", "pin": 34,
            "ori_rpy_deg": [0, 0, 360-25],
            "pos_xyz_m": [0.05, -0.05, 0]
        },
        {
            "label": "back_left", "pin": 35,
            "ori_rpy_deg": [0, 0, 90],
            "pos_xyz_m": [-0.07, 0.05, 0]
        },
        {
            "label": "back_right", "pin": 36,
            "ori_rpy_deg": [0, 0, 270],
            "pos_xyz_m": [-0.07, -0.05, 0]
        },
    ]
}


###############################################################################
def do_install_requirements():
    try:
        import logging  # noqa
    except Exception:
        import upip
        upip.install("micropython-logging")


###############################################################################
def do_start_network():
    import network
    sta_if = network.WLAN(network.STA_IF)

    if CONFIG["network"]["ssid"].startswith("XX") and \
       CONFIG["network"]["ssid"].endswith("XX"):
        raise Exception(
            "Please edit your network configuration in the ./{} file, "
            "then re-upload the file to the ESP32 board.".format(
                CONFIG_FILE))

    led_pin = machine.Pin(PIN_CONFIG["led"]["status"], machine.Pin.OUT)
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
    ifconfig = sta_if.ifconfig()
    print('Network connected. Config: ', ifconfig)
    CONFIG["network"]["hadabot_ip_address"] = ifconfig[0]
    led_pin.on()


###############################################################################
def do_read_config():
    global CONFIG, PIN_CONFIG
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

        # LED?
        try:
            led_gpio = CONFIG["pin_config"]["led"]["status"]
            PIN_CONFIG["led"]["status"] = led_gpio

            # Turn off core LED
            led_pin = machine.Pin(PIN_CONFIG["led"]["status"], machine.Pin.OUT)
            led_pin.off()
        except Exception:
            pass

    else:
        raise Exception("Could not find a {} config file".format(CONFIG_FILE))


###############################################################################
def do_setup():
    import logging

    logger = logging.getLogger(__name__)
    logger.info("Welcome to Hadabot - www.hadabot.com")

    webrepl.start()


###############################################################################
def do_initial_prep():
    # Turn off all motors
    if True:
        for motor_pin in [
                PIN_CONFIG["left"]["motor"]["pwm"],
                PIN_CONFIG["right"]["motor"]["pwm"]]:
            pmotor = machine.PWM(machine.Pin(motor_pin))
            pmotor.duty(0)
        for motor_pin in [
                PIN_CONFIG["left"]["motor"]["fr"],
                PIN_CONFIG["right"]["motor"]["fr"]]:
            pmotor = machine.Pin(motor_pin, machine.Pin.OUT)
            pmotor.off()

    # Turn off core LED
    led_pin = machine.Pin(PIN_CONFIG["led"]["status"], machine.Pin.OUT)
    led_pin.off()

    # Need to move some files over to root directory
    core_files = ["webrepl_cfg.py", CONFIG_FILE, "main.py", "ssd1306.py"]
    need_copy = False
    need_reset = False

    # If any of the files don't exist on the ESP32's root level then we need
    # to update the firmware from uhadabot directory
    for core_file in core_files:
        if core_file not in os.listdir():
            print("Need to move {} to root folder".format(core_file))
            need_copy = True

    # If all of the core files exist in the uhadabot directory then we
    # also need to copy them over - not it's all-or-no-copy
    ls_uhadabot = os.listdir("uhadabot")
    if all([(core_file in ls_uhadabot) for core_file in core_files]):
        print(
            "All core files in uhadabot, so we will update the entire folder")
        need_copy = True

    if need_copy:
        for core_file in core_files:
            if core_file not in ls_uhadabot and \
               core_file not in os.listdir():
                raise Exception(
                    "Could not find {} in / or /uhadabot folders".format(
                        core_file))

            if core_file in ls_uhadabot:
                # Move the file from libary folder out to root
                print(
                    "Moving {} from uhadabot to root".format(core_file))
                os.rename(
                    "/uhadabot/{}".format(core_file), "/{}".format(core_file))
            need_reset = True

    if need_reset:
        machine.reset()


###############################################################################
# Initial boot calls
do_initial_prep()
do_read_config()
do_start_network()
do_install_requirements()
do_setup()
