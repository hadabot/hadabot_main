#! /bin/bash

# Build the hadabot containers
cd hadabot_main/docker
echo "We will be building the Hadabot Docker containers. This may take around 30 min or so depending on your network speed..."
docker-compose up -d

# Instructions on how to get the ESP32 flashed
cd ~
echo "You are now read to flash the ESP32."
echo "Connect your ESP32 with a quality micro-USB"
echo "Type the following in your terminal below - one command at a time:"
echo ""
echo "$ cd hadabot_main/content/p7/firmware"
echo "$ esptool.py --port /dev/ttyUSB0 erase_flash"
echo "$ esptool.py --chip esp32 --port /dev/ttyUSB0 --baud 460800 write_flash -z 0x1000 ~/esp32-micropython.bin"
echo "$ ampy --port /dev/ttyUSB0 run clean_firmware.py"

