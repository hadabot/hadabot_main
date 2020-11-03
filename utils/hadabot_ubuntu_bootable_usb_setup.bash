#! /bin/bash

# With a terminal from Ubuntu, run:
# $ wget -O hadabot_setup.bash https://raw.githubusercontent.com/hadabot/hadabot_main/master/utils/hadabot_ubuntu_bootable_usb_setup.bash
# $ ./hadabot_setup.bash

echo "-----"
echo "Updating system first."
echo "Don't go away! The update may take 5-15 minutes and we need you to stick around to help with setup."
echo "Got it?"
echo "[Press any key to continue]"
read -s -n 1

echo "-----"
echo "You may need to enter your password for super user access to update the system."
sudo add-apt-repository universe
sudo apt-get update
sudo apt-get upgrade -y
sudo apt-get install -y git

# Clone the hadabot repo
echo "Cloning Hadabot Github repo..."
cd ~
git clone https://github.com/hadabot/hadabot_main.git
echo "Done cloning Hadabot Github repo."

# Set up the ESP32
cd ${HOME}/hadabot_main/utils
./hadabot_esp32_setup.bash

# Install docker
echo "----"
echo "We will now be installing Docker and building the Hadabot Docker containers."
echo ""
echo "We may (or may not) need your system password again so stick around for a minute or so."
echo ""
echo "Once you see a stream of commands kick off, you are good to go get your coffee. This step will take a while (longer than 1 hour)."
echo ""
echo "Sound good?"
echo "[Press any key to continue]"
read -s -n 1
cd ${HOME}/hadabot_main/utils
./hadabot_docker_container_setup.bash

echo "----"
echo "If there were no errors, then your setup is complete."
echo ""
echo "Let's power up your Hadabot Turtle, in the order below."
echo ""
echo "(Step 1) Power up the ESP32 - plug one of the red-battery-pack power wires to the 'female-ended power wire' for your ESP32 board."
echo ""
echo "(Step 2) Power up the motor controller - plug the other red-battery-pack power wire to the 'female-ended power wire' for your motor control board."
echo ""
echo "NOTE: The ESP32 needs to be powered up *before* the motor controller!"
echo ""
echo "Done powering up the Hadabot Turtle?"
echo "[Press any key to continue]"
read -s -n 1

echo "----"
echo "Reset your ESP32 by pressing/holding the 'EN' button for 2 seconds."
echo ""
echo "Done pressing/holding/releasing the 'EN' button on the ESP32?"
echo "[Press any key to continue]"
read -s -n 1

# Open teleop instructions
echo "----"
echo "The ESP32 board's blue on-board LED should blink then stay lit if the board has managed to connect to your Wi-Fi and the Hadabot ROS 2 software stack."
echo ""
echo "Do you see the blue on-board LED lit solid (if not, email hi@hadabot.com and we'll help you troubleshoot)?"
echo "[Press any key if the blue LED is lit, else Ctrl-C to exit]"
read -s -n 1

echo "----"
echo "When you're ready, we'll open the Hadabot Teleop webpage for you to teleoperate your Hadabot Turtle robot!"
echo ""
echo "Ready?"
echo "[Press any key to continue]"
read -s -n 1
firefox https://www.hadabot.com/ros2-hadabot-teleop.html?step=compile-unicycle-ros2-code
