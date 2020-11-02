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
echo "Installing ESP32 tools..."
echo "You may be prompted for your password to install some components."
echo "Please stick around since we need to ask you some questions."
echo "[Press any key to continue]"
read -s -n 1

# Install necessary python components
sudo apt-get install -y python3-pip net-tools

# Install the tools needed to flash the ESP32
echo "export PATH=$PATH:${HOME}/.local/bin" >> ~/.bashrc
export PATH=${PATH}:${HOME}/.local/bin
pip3 install -U esptool adafruit-ampy
wget -O esp32-micropython.bin https://micropython.org/resources/firmware/esp32-idf3-20200902-v1.13.bin
sudo usermod -aG dialout ${USER}
sudo chmod a+rw /dev/ttyUSB0

# Clone the hadabot repo
git clone https://github.com/hadabot/hadabot_main.git

# Instructions on how to get the ESP32 flashed
cd ~
cd hadabot_main/content/p7/firmware
esptool.py --port /dev/ttyUSB0 erase_flash
esptool.py --chip esp32 --port /dev/ttyUSB0 --baud 460800 write_flash -z 0x1000 ~/esp32-micropython.bin
ampy --port /dev/ttyUSB0 run clean_firmware.py
ampy --port /dev/ttyUSB0 put uhadabot
#ampy --port /dev/ttyUSB0 put boot.py
