#! /bin/bash

bootfile=./boot.py
if [ -f "$bootfile" ]; then
    echo "Attempting to flash the firmware found in `pwd`"
else
    echo "Hadabot firmware not found in `pwd`. Quitting."
    exit 1;
fi

devfile=/dev/ttyUSB0
if [ -e "$devfile" ]; then
    echo ""
else
    echo "Didn't find a ESP32 board mounted as $devfile. Did you plug in the ESP32? Quitting."
    exit 1;
fi

hostip=`./../../../utils/hadabot_get_system_ip.bash`
echo "----"
echo "Is there only one (1) IP address shown?"
echo "${hostip}"
echo "[Press any key if there's only one (1) IP address, else Ctrl-C to exit]"
read -n 1 -s

echo "----"
echo "We need to configure the firmware so that the ESP32 can connect to your Wi-Fi network..."
echo "What is your Wi-Fi SSID?"
read wifissid
echo "What is your Wi-Fi password?"
read wifipassword

echo "{
  \"ros2_web_bridge_ip_addr\": \"${hostip}\",
  \"network\": {
    \"ssid\": \"${wifissid}\",
    \"password\": \"${wifipassword}\"
  }
}" > ./uhadabot/hb.json

echo "----"
echo "Cleaning old firmware off the ESP32..."
export PATH=${PATH}:${HOME}/.local/bin
sudo chmod a+rw /dev/ttyUSB0
ampy --port /dev/ttyUSB0 run clean_firmware.py >> /dev/null
echo "----"
echo "Old firmware cleaned off of the ESP32."
echo ""
echo "Please press/hold the 'EN' button on the ESP32 board for 2 seconds, then release (to reset the board)."
echo "Done pressing/holding/releasing the 'EN' button?"
echo "[Press any key to continue]"
read -n 1 -s

echo "Copying new firmware to the ESP32 (patience please)..."
ampy --port /dev/ttyUSB0 put uhadabot
echo "Almost done..."
ampy --port /dev/ttyUSB0 put boot.py
echo "Done copying new firmware to the ESP32"

