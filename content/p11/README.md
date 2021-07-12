# Run the Navigation2 Go-To-Goal example with the Hadabot Turtle robot

## 1. Prep your Hadabot Software Stack (in this browser-based VSCode)

Update the ROS 2, system packages:

1. Open a terminal by clicking on the upper left menu bar icon -> Terminal -> New Terminal.
1. In the terminal, type:

```
sudo apt-get update
sudo apt-get upgrade -y
```

(The update might take a while...)

-------

## 2. Upload the firmware for this example to your Hadabot Turtle (on your HOST machine)

All steps done on your __host__ machine:

#### a. Set up the Turtle's ESP32 network configuration

```
$ cd hadabot_main/content/p9/firmware
$ cp uhadabot/hadabot_config_template.json uhadabot/hb.json
```

Enter your network info by editing the uhadabot/hb.json file:

1. ros2_web_bridge_ip_addr:
    1. Enter the IP address of your host development system (wrapped with double-quotes since JSON only recognizes strings).
1. network:
    1. Enter the SSID and password of your network (also wrapped in double-quotes).

Confirm the network settings:

```
$ cat uhadabot/hb.json
```

#### b. Flash the Turtle's ESP32

1. Press the "Boot" button on the ESP32 board to disconnect from the ROS system.
    1. The blue on-board LED should turn off.
1. Unplug your Turtle's ESP32 and Motor Driver from the batteries.
1. Using a quality micro-USB cable, plug the ESP32 to your development computer.
1. Then run the following to upload the firmware:

```
$ ampy --port /dev/<the_esp32_usb_port> run clean_firmware.py
$ ampy --port /dev/<the_esp32_usb_port> put uhadabot
$ ampy --port /dev/<the_esp32_usb_port> put boot.py
```

__IMPORTANT__: you must FIRST run 'ampy --port XX put uhadabot', since boot.py will use files from the /uhadabot folder.

--------
--------

## 3. Compile and run the ROS 2 code (in this browser-based VSCode)

Follow these steps to compile the ROS 2 source code:

1. Open a terminal by clicking on the upper left menu bar icon -> Terminal -> New Terminal.

1. In the terminal, type: 

```
source /opt/ros/foxy/setup.bash
cd hadabot_ws
colcon build --symlink-install 
```

3. Once the build completes, we need to open up a browser-based VNC window to kick off the application. Open a browser-based VNC environment via http://localhost:9124/

## In the __browser-based VNC__ tab:

1. Kick off a bash terminal:
    1. Left-click the lower-left "chevron-like" system icon -> System Tools -> LXTerminal.

1. In the new LXTerminal bash terminal:
    1. `$ cd hadabot_main/content/p9/hadabot_ws/`
    1. `$ source install/setup.bash`
    1. `$ cd launch`
    1. `$ ros2 launch hadabot_nav2_launch.py`

1. The rviz window should appear

1. In the rviz application:
    1. Click on "Navigation2 Goal" button (in the top main menu bar).
    1. Click-n-hold somewhere in the white patch, drag to specify a direction, you should see a large green arrow appear. That is the goal pose - position and orientation which Navigation2 will drive our Hadabot Turtle towards the goal.

1. Watch the Hadabot Turtle move to the goal pose.
    1. Each grid line in rviz is 1 meter.

-----------
-----------

## Misc

To send Nav2 goal using the ros2 cli
```
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "
{
pose: {
  header: {
    stamp: {
      sec: 1607032613,
      nanosec: 0,
	  },
    frame_id: map
    },
  pose: {
    position: {
      x: 6.0,
      y: 5.0,
      z: 0.0
      },
    orientation: {
      x: 0.0,
      y: 0.0,
      z: 0.0,
      w: 1.0
      }
    }
  }
}
"
```

### With AWS RoboMaker

1. Clone hadabot_main, set up
    1. `cd ~`
    1. `git clone https://github.com/hadabot/hadabot_main.git`
    1. `export RMW_IMPLEMENTATION="rmw_cyclonedds_cpp"`
    1. `sudo apt-get install -y ros-foxy-rmw-cyclonedds-cpp ros-foxy-navigation2 ros-foxy-nav2-bringup`
    1. `cd hadabot_main/content/p9/hadabot_ws/`
    1. `colcon build --symlink-install`
1. [Make VPC for RoboMaker Cloud 9](https://docs.aws.amazon.com/cloud9/latest/user-guide/vpc-settings.html#vpc-settings-create-vpc)
1. [Launch RoboMaker Development Environment](https://console.aws.amazon.com/robomaker/home)
1. Install ros2_web_bridge
    1. `cd ~`
    1. `curl -sL https://deb.nodesource.com/setup_12.x -o nodesource_setup.sh`
    1. `sudo /bin/bash nodesource_setup.sh`
    1. `nvm install v12.22.1`
        1. // May not need `sudo apt-get install -y nodejs`
    1. `git clone https://github.com/RobotWebTools/ros2-web-bridge.git`
    1. `cd ros2-web-bridge`
    1. `git checkout 0.3.1 -b rel_branch`
    1. `npm install rclnodejs@0.18.2`
    1. `npm install`
    1. `export RMW_IMPLEMENTATION="rmw_cyclonedds_cpp"`
    1. `node ./bin/rosbridge.js`
1. Build p9/content/hadabot_ws
1. Launch Virtual Desktop (might have to let Firefox open a new tab)
1. Run in Virtual Desktop byobu terminal
    1. `export RMW_IMPLEMENTATION="rmw_cyclonedds_cpp"`
    1. `cd hadabot_main/content/p9/hadabot_ws/`
    1. `$ source install/setup.bash`
    1. `$ cd launch`
    1. `$ ros2 launch hadabot_nav2_launch.py`