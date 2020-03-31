import os
import machine


def del_folder(folder):
    for f in os.ilistdir(folder):
        ftype = f[1]
        fname = f[0]
        full_path_name = "{}/{}".format(folder, fname)

        # Is a folder
        if ftype == 0x4000:
            del_folder(full_path_name)
        else:
            # Delete file
            os.remove(full_path_name)

    # Remove the folder
    os.rmdir(folder)


def main(argv):
    os.chdir("/")
    try:
        del_folder("/uhadabot")
    except Exception:
        print("FYI: did not find the uhadabot firmware folder on the ESP32.")

    # Create a stub boot.py for MicroPython
    os.remove("boot.py")
    with open("boot.py", "w") as f:
        f.write("""import machine
led_pin = machine.Pin(2, machine.Pin.OUT)
led_pin.off()
        """)

    reset_irq = machine.Timer(-1)
    reset_irq.init(mode=machine.Timer.ONE_SHOT, period=200,
                   callback=lambda t: machine.reset())

    print("""
Now you can copy the latest firmware on your system to the Hadabot's ESP32

$ ampy --port /dev/<USB_PORT> put uhadabot

$ ampy --port /dev/<USB_PORT> put boot.py

Then press-release the "EN" button to reset the ESP32.

""")


if __name__ == "__main__":
    main(None)
