#! /bin/bash

# With a terminal from Ubuntu, run "wget -O hadabot_setup.bash https://raw.githubusercontent.com/hadabot/hadabot_main/master/utils/hadabot_ubuntu_usb_bootable_setup.bash"

echo "-----"
echo "Updating system first... you may need to enter your password for super user access to update the system."
echo "-----"
sudo add-apt-repository universe
sudo apt-get update
sudo apt-get upgrade -y

# Install docker
echo "-----"
echo "Installing docker..."
echo "-----"
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
sudo usermod -aG docker ${USER}

# "Try Ubuntu" does not like it when docker users aufs as the Docker storage driver
# When using default aufs, "docker run hello-world" will cause a weird invalid arg aufs error.
#sudo cp /lib/systemd/system/docker.service /lib/systemd/system/docker.service.orig
#sed 's/ExecStart=\/usr\/bin\/dockerd -H/ExecStart=\/usr\/bin\/dockerd --storage-driver=devicemapper -H/g' /lib/systemd/system/docker.service.orig > docker.service
#sudo mv docker.service /lib/systemd/system/docker.service
#sudo systemctl daemon-reload
#sudo systemctl restart docker.service

# Install docker-compose
echo "-----"
echo "Installing docker-compose..."
echo "-----"
sudo curl -L "https://github.com/docker/compose/releases/download/1.27.4/docker-compose-$(uname -s)-$(uname -m)" -o /usr/local/bin/docker-compose
sudo chmod +x /usr/local/bin/docker-compose

# Build the hadabot containers
cd hadabot_main/docker
echo "------"
echo "We will be building the Hadabot Docker containers. This may take around 30 min or so depending on your network speed..."
echo "------"
sudo docker-compose up -d

# Open teleop instructions
firefox https://www.hadabot.com/ros2-hadabot-teleop.html?step=compile-unicycle-ros2-code
