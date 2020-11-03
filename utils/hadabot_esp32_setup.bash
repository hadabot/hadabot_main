#! /bin/bash

echo ""
echo "------------------"
echo ""
echo "Let's set up the Hadabot Bootable USB setup."
echo ""
echo "Using a quality micro-USB cable, please connect your ESP32 from your Hadabot Turtle to your computer."
echo ""
echo "Done?"
echo "[Press any key to continue]"
read -s -n 1

echo "-----"
echo "Setting up the ESP32 tools and configurations..."
echo ""
echo "Until we tell you to go get coffee, please stick around since (1) we need you to enter your system password again, and (2) we will need to ask you some network questions."
echo "[Press any key to continue]"
read -s -n 1

# Install necessary python components
sudo apt-get install -y python3-pip net-tools git

# Install the tools needed to flash the ESP32
echo "export PATH=$PATH:${HOME}/.local/bin" >> ~/.bashrc
export PATH=${PATH}:${HOME}/.local/bin
pip3 install -U esptool adafruit-ampy
wget -O ${HOME}/esp32-micropython.bin https://micropython.org/resources/firmware/esp32-idf3-20200902-v1.13.bin
sudo usermod -aG dialout ${USER}
sudo chmod a+rw /dev/ttyUSB0

# Flash MicroPython on to the ESP32
echo "----"
echo "We are going to cleanly erase the ESP32."
echo ""
echo "Get your ESP32 board handy."
echo ""
echo "When you see 'Connecting....____', please press/hold the 'BOOT' button on the ESP32 board. You can release the button after 3-5 seconds (or until you see 'Erasing flash...')."
echo "Got it?"
echo "[Press any key to continue]"
read -s -n 1
esptool.py --port /dev/ttyUSB0 erase_flash
echo "----"
echo "Now, we are going to flash MicroPython to the ESP32"
echo ""
echo "Again, when you see 'Connecting....____', please press/hold the 'BOOT' button on the ESP32 board. Like last time, you can release the button after 3-5 seconds (or until you see 'Writing flash...')."
echo "Sounds good?"
echo "[Press any key to continue]"
read -s -n 1
esptool.py --chip esp32 --port /dev/ttyUSB0 --baud 460800 write_flash -z 0x1000 ${HOME}/esp32-micropython.bin

# Clone the hadabot repo
echo "Cloning Hadabot Github repo..."
cd ~
rm -rf hadabot_main
git clone https://github.com/hadabot/hadabot_main.git
echo "Done cloning Hadabot Github repo."

# Flash the firmware
cd ~
cd hadabot_main/content/p7/firmware
./../../../utils/hadabot_flash_firmware.bash

echo "----"
echo "New firmware copied to the ESP32."
echo ""
echo "Please unthether/disconnect your ESP32/Hadabot Turtle from the micro-USB cable connected to your computer."
echo ""
echo "Done disconnecting the micro-USB cable from the ESP32?"
echo "[Press any key to continue]"
read -n 1 -s
