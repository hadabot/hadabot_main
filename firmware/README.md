# Setup Hadabot Firmware on the Hadabot ESP32 controller board

## Install Micropython on the ESP32

1. Download the [Micropython 1.12](https://micropython.org/resources/firmware/esp32-idf3-20191220-v1.12.bin)
    1. Or you can try the [latest Micropython for ESP32](https://micropython.org/download#esp32).
1. [Flash Micropython on to your ESP32 using esptool](https://docs.micropython.org/en/latest/esp32/tutorial/intro.html#deploying-the-firmware) - summary:
    1. In your python virtualenv or equivalent, run `pip3 install esptool`
	    1. If you are using virtualenv, you may need to reactivate the virtualenv to be able to "see" esptool.py script.
	1. Connect the ESP32 to your computer, note the USB port (ie /dev/ttyUSB0 for Linux, /dev/tty.SLAB_USBtoUART for MacOS, etc)
	1. While holding down the "Boot" button on the ESP32 board (or else you'll get a _Failed to connect to ESP32_ error). NOTE: You can let go once the esptool.py connects.
	    1. `esptool.py --port /dev/<ESP32_USB_PORT> erase_flash`
	    1. `esptool.py --chip esp32 --port /dev/<ESP32_USB_PORT> --baud 460800 write_flash -z 0x1000 /<Your download folder>/esp32-idf3-20191220-v1.12.bin`
		
## Install the Hadabot firmware on ESP32

1. In your python virtualenv or equivalent python-sandbox, install a file manager tool for the ESP32:
    1. `pip3 install adafruit-ampy`
	    1. If you are using virtualenv, you may need to reactivate the virtualenv to be able to "see" ampy script.
1. Inside the firmware folder - `cd hadabot_main/firmware`
    1. Setup the ESP32 configurations from a template:
	    1. `cd uhadabot`
			1. `cp ./hadabot_config_template.json ./hb.json`
			1. Open ./hb.json and edit the network SSID and PWD configurations
			1. Set the _ros2\_web\_bridge\_ip\_addr_ to the IP address of the host machine on which you started our Hadabot _docker-compose up -d_ docker stack (likely the machine you are working on right now).
    1. Upload the Hadabot firmware code to the ESP32
	    1. `cd ..` to go back to _hadabot\_main/firmware_
        1. (OPTIONAL - To clean the existing code first)
	        1. `ampy --port /dev/tty.<ESP32_USB_PORT> rmdir uhadabot`
        1. `ampy --port /dev/tty.<ESP32_USB_PORT> put uhadabot`
        1. `ampy --port /dev/tty.<ESP32_USB_PORT> put boot.py`
	1. Reset the board by pressing the "EN" button for a sec and releasing it.
	    1. (OPTIONAL - but not as reliable as pushing "EN") `ampy --port /dev/tty.<ESP32_USB_PORT> reset`
		    1. Onboard LED will blink while setting up the network, and stay lit when network is established.
		
## To See the WebREPL

First find the Websocket URL for the Hadabot control board's REPL server:

`ampy --port /dev/tty.<ESP32_USB_PORT> run ./scripts/get_webrepl_url.py`


## Pinouts for the Hadabot ESP32 controller board

[Pinout diagram](https://circuits4you.com/wp-content/uploads/2018/12/ESP32-Pinout.jpg)

[Pin explanation](https://circuits4you.com/2018/12/31/esp32-devkit-esp32-wroom-gpio-pinout/)
	
## Launching a python3 REPL terminal

For MacOS and Linux

1. Install picocom
    1. MacOS `brew install picocom`
	1. Linux `apt install picocom`
1. `picocom /dev/<ESP32\_USB\_PORT> -b115200`
1. Hit enter a couple of times to see the ">>>" prompt
1. Picocom commands:
    1. To quit - Ctrl-a, Ctrl-q
	1. To soft reset - Ctrl-d

For Windows

1. Install PuTTY
1. ... (Work-in-progress)

### Blink the onboard LED

In the REPL:

`>>> import machine`

`>>> pin = machine.Pin(2, machine.Pin.OUT)`

`>>> pin.on()`

`>>> pin.off()`

For modules and function definitions, [refer to the entire Micropython library spec](https://docs.micropython.org/en/latest/library/index.html).
