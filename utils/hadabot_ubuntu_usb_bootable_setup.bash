#! /bin/bash

# With a terminal from Ubuntu, run "wget -O hadabot_ubuntu_usb_bootable_setup.bash https://raw.githubusercontent.com/hadabot/hadabot_main/master/utils/hadabot_ubuntu_usb_bootable_setup.bash"

sudo add-apt-repository universe
sudo apt-get update

# Install docker
sudo apt-get install -y \
    apt-transport-https \
    ca-certificates \
    curl \
    gnupg-agent \
    software-properties-common
curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo apt-key add -
sudo add-apt-repository \
   "deb [arch=amd64] https://download.docker.com/linux/ubuntu \
   $(lsb_release -cs) \
   stable"
sudo apt-get update
sudo apt-get install -y docker-ce docker-ce-cli containerd.io
sudo usermod -aG docker ubuntu

# Install docker-compose
sudo curl -L "https://github.com/docker/compose/releases/download/1.27.4/docker-compose-$(uname -s)-$(uname -m)" -o /usr/local/bin/docker-compose
sudo chmod +x /usr/local/bin/docker-compose

# Install necessary python components
sudo apt-get install -y python3-pip

# Install the tools needed to flash the ESP32
echo "export PATH=$PATH:~/.local/bin" >> ~/.bashrc
pip3 install -U esptool adafruit-ampy
wget -O esp32-micropython.bin https://micropython.org/resources/firmware/esp32-idf3-20200902-v1.13.bin
sudo usermod -aG dialout ubuntu

# Clone the hadabot repo
git clone https://github.com/hadabot/hadabot_main.git

# Login again to another shell to activate dialout and docker permission group
su ubuntu

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

