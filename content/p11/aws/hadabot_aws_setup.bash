source ~/.bash_profile
sudo apt-get update; sudo apt-get upgrade -y
export RMW_IMPLEMENTATION="rmw_cyclonedds_cpp"
sudo apt-get install -y ros-foxy-rmw-cyclonedds-cpp ros-foxy-navigation2 ros-foxy-nav2-bringup
colcon build --symlink-install
cd ~
nvm install v12.22.1
git clone https://github.com/RobotWebTools/ros2-web-bridge.git
cd ros2-web-bridge
git checkout 0.3.1 -b rel_branch
npm install rclnodejs@0.18.2
npm install
node ./bin/rosbridge.js