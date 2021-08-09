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

--------

## With AWS RoboMaker

1. [Make VPC for RoboMaker Cloud 9](https://docs.aws.amazon.com/cloud9/latest/user-guide/vpc-settings.html#vpc-settings-create-vpc)
1. [Launch RoboMaker Development Environment](https://console.aws.amazon.com/robomaker/home)
    1. [Find the relevant 'cloud-9' security group](https://console.aws.amazon.com/vpc/home?region=us-east-1#securityGroups:)
        1. Edit 'Inbound Rules' - Add 'All TCP' for '0.0.0.0/0'
1. Clone hadabot_main, set up
    1. `cd ~`
    1. `git clone https://github.com/hadabot/hadabot_main.git`
    1. `sudo apt-get update; sudo apt-get upgrade -y`
    1. `export RMW_IMPLEMENTATION="rmw_cyclonedds_cpp"`
    1. `sudo apt-get install -y ros-foxy-rmw-cyclonedds-cpp ros-foxy-navigation2 ros-foxy-nav2-bringup`
    1. `cd hadabot_main/content/p11/hadabot_ws/`
    1. `colcon build --symlink-install`
1. Install ros2_web_bridge
    1. `cd ~`
    1. May not need the next steps since there's nvm already
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
1. In each new terminal you open
    1. `export RMW_IMPLEMENTATION="rmw_cyclonedds_cpp"; cd ~/hadabot_main/content/p11/hadabot_ws/; source install/setup.bash`
1. To connect to insecure Websocket ROS bridge from [Hadabot Dashboard](https://www.hadabot.com/tools/hadabot-dashboard.html)
    1. Click on 'padlock' to the left of the URL
    1. Site Settings -> Insecure Content -> "Allow"

# Latency measurements

1. Terminals running:
    1. `ros2 run hadabot_latency talker_inertia_stamped`
    1. `ros2 run hadabot_latency listener_with_topic_statistics`
    1. `ros2 topic echo /statistics/ping_inertia_stamped`


## InertiaStamped 

ubuntu:~/hadabot_main/content/p11/hadabot_ws (master) $ ros2 topic echo /statistics/ping_inertia_stamped 
1626297503.007929 [0]       ros2: using network interface ens3 (udp/10.0.0.134) selected arbitrarily from: ens3, docker0
measurement_source_name: minimal_subscriber_with_topic_statistics
metrics_source: message_age
unit: ms
window_start:
  sec: 1626297499
  nanosec: 166616332
window_stop:
  sec: 1626297509
  nanosec: 166439122
statistics:
- data_type: 1
  data: 658.2999999999998
- data_type: 3
  data: 917.0
- data_type: 2
  data: 411.0
- data_type: 5
  data: 60.0
- data_type: 4
  data: 139.99098780516792
---
measurement_source_name: minimal_subscriber_with_topic_statistics
metrics_source: message_period
unit: ms
window_start:
  sec: 1626297499
  nanosec: 166616332
window_stop:
  sec: 1626297509
  nanosec: 166439122
statistics:
- data_type: 1
  data: 500.45762711864415
- data_type: 3
  data: 951.0
- data_type: 2
  data: 324.0
- data_type: 5
  data: 59.0
- data_type: 4
  data: 195.45967740860138
---
measurement_source_name: minimal_subscriber_with_topic_statistics
metrics_source: message_age
unit: ms
window_start:
  sec: 1626297509
  nanosec: 166439122
window_stop:
  sec: 1626297519
  nanosec: 166150033
statistics:
- data_type: 1
  data: 650.8888888888889
- data_type: 3
  data: 917.0
- data_type: 2
  data: 411.0
- data_type: 5
  data: 81.0
- data_type: 4
  data: 138.28098072269242
---
measurement_source_name: minimal_subscriber_with_topic_statistics
metrics_source: message_period
unit: ms
window_start:
  sec: 1626297509
  nanosec: 166439122
window_stop:
  sec: 1626297519
  nanosec: 166150033
statistics:
- data_type: 1
  data: 495.58750000000015
- data_type: 3
  data: 951.0
- data_type: 2
  data: 324.0
- data_type: 5
  data: 80.0
- data_type: 4
  data: 191.49293027093717
---
measurement_source_name: minimal_subscriber_with_topic_statistics
metrics_source: message_age
unit: ms
window_start:
  sec: 1626297519
  nanosec: 166150033
window_stop:
  sec: 1626297529
  nanosec: 165982129
statistics:
- data_type: 1
  data: 654.28
- data_type: 3
  data: 917.0
- data_type: 2
  data: 411.0
- data_type: 5
  data: 100.0
- data_type: 4
  data: 138.69138978321618
---
measurement_source_name: minimal_subscriber_with_topic_statistics
metrics_source: message_period
unit: ms
window_start:
  sec: 1626297519
  nanosec: 166150033
window_stop:
  sec: 1626297529
  nanosec: 165982129
statistics:
- data_type: 1
  data: 500.75757575757586
- data_type: 3
  data: 951.0
- data_type: 2
  data: 324.0
- data_type: 5
  data: 99.0
- data_type: 4
  data: 195.11768481471992
---
measurement_source_name: minimal_subscriber_with_topic_statistics
metrics_source: message_age
unit: ms
window_start:
  sec: 1626297529
  nanosec: 165982129
window_stop:
  sec: 1626297539
  nanosec: 165728080
statistics:
- data_type: 1
  data: 653.075
- data_type: 3
  data: 917.0
- data_type: 2
  data: 402.0
- data_type: 5
  data: 120.0
- data_type: 4
  data: 137.73508161805884
---
measurement_source_name: minimal_subscriber_with_topic_statistics
metrics_source: message_period
unit: ms
window_start:
  sec: 1626297529
  nanosec: 165982129
window_stop:
  sec: 1626297539
  nanosec: 165728080
statistics:
- data_type: 1
  data: 498.7563025210084
- data_type: 3
  data: 951.0
- data_type: 2
  data: 95.0
- data_type: 5
  data: 119.0
- data_type: 4
  data: 199.22425042167066
---
measurement_source_name: minimal_subscriber_with_topic_statistics
metrics_source: message_age
unit: ms
window_start:
  sec: 1626297539
  nanosec: 165728080
window_stop:
  sec: 1626297549
  nanosec: 165436027
statistics:
- data_type: 1
  data: 646.5785714285714
- data_type: 3
  data: 917.0
- data_type: 2
  data: 402.0
- data_type: 5
  data: 140.0
- data_type: 4
  data: 136.84334462949872
---
measurement_source_name: minimal_subscriber_with_topic_statistics
metrics_source: message_period
unit: ms
window_start:
  sec: 1626297539
  nanosec: 165728080
window_stop:
  sec: 1626297549
  nanosec: 165436027
statistics:
- data_type: 1
  data: 497.9784172661871
- data_type: 3
  data: 951.0
- data_type: 2
  data: 95.0
- data_type: 5
  data: 139.0
- data_type: 4
  data: 197.0549157518162
---
measurement_source_name: minimal_subscriber_with_topic_statistics
metrics_source: message_age
unit: ms
window_start:
  sec: 1626297549
  nanosec: 165436027
window_stop:
  sec: 1626297559
  nanosec: 165224002
statistics:
- data_type: 1
  data: 649.2500000000003
- data_type: 3
  data: 917.0
- data_type: 2
  data: 402.0
- data_type: 5
  data: 160.0
- data_type: 4
  data: 136.8188126684339
---
measurement_source_name: minimal_subscriber_with_topic_statistics
metrics_source: message_period
unit: ms
window_start:
  sec: 1626297549
  nanosec: 165436027
window_stop:
  sec: 1626297559
  nanosec: 165224002
statistics:
- data_type: 1
  data: 499.63522012578613
- data_type: 3
  data: 951.0
- data_type: 2
  data: 95.0
- data_type: 5
  data: 159.0
- data_type: 4
  data: 197.38664994662832
---
measurement_source_name: minimal_subscriber_with_topic_statistics
metrics_source: message_age
unit: ms
window_start:
  sec: 1626297559
  nanosec: 165224002
window_stop:
  sec: 1626297569
  nanosec: 164993449
statistics:
- data_type: 1
  data: 642.7166666666669
- data_type: 3
  data: 917.0
- data_type: 2
  data: 176.0
- data_type: 5
  data: 180.0
- data_type: 4
  data: 143.81003190938313
---
measurement_source_name: minimal_subscriber_with_topic_statistics
metrics_source: message_period
unit: ms
window_start:
  sec: 1626297559
  nanosec: 165224002
window_stop:
  sec: 1626297569
  nanosec: 164993449
statistics:
- data_type: 1
  data: 496.55865921787705
- data_type: 3
  data: 951.0
- data_type: 2
  data: 84.0
- data_type: 5
  data: 179.0
- data_type: 4
  data: 202.5934936123769
---
measurement_source_name: minimal_subscriber_with_topic_statistics
metrics_source: message_age
unit: ms
window_start:
  sec: 1626297569
  nanosec: 164993449
window_stop:
  sec: 1626297579
  nanosec: 164757517
statistics:
- data_type: 1
  data: 644.1150000000005
- data_type: 3
  data: 917.0
- data_type: 2
  data: 176.0
- data_type: 5
  data: 200.0
- data_type: 4
  data: 142.8328805807683
---
measurement_source_name: minimal_subscriber_with_topic_statistics
metrics_source: message_period
unit: ms
window_start:
  sec: 1626297569
  nanosec: 164993449
window_stop:
  sec: 1626297579
  nanosec: 164757517
statistics:
- data_type: 1
  data: 499.7939698492462
- data_type: 3
  data: 951.0
- data_type: 2
  data: 84.0
- data_type: 5
  data: 199.0
- data_type: 4
  data: 203.04249317320017
---
measurement_source_name: minimal_subscriber_with_topic_statistics
metrics_source: message_age
unit: ms
window_start:
  sec: 1626297579
  nanosec: 164757517
window_stop:
  sec: 1626297589
  nanosec: 164521929
statistics:
- data_type: 1
  data: 644.4590909090912
- data_type: 3
  data: 917.0
- data_type: 2
  data: 176.0
- data_type: 5
  data: 220.0
- data_type: 4
  data: 141.5650931521382
---
measurement_source_name: minimal_subscriber_with_topic_statistics
metrics_source: message_period
unit: ms
window_start:
  sec: 1626297579
  nanosec: 164757517
window_stop:
  sec: 1626297589
  nanosec: 164521929
statistics:
- data_type: 1
  data: 498.9497716894977
- data_type: 3
  data: 951.0
- data_type: 2
  data: 84.0
- data_type: 5
  data: 219.0
- data_type: 4
  data: 201.38044650637679
---
measurement_source_name: minimal_subscriber_with_topic_statistics
metrics_source: message_age
unit: ms
window_start:
  sec: 1626297589
  nanosec: 164521929
window_stop:
  sec: 1626297599
  nanosec: 164252629
statistics:
- data_type: 1
  data: 643.429166666667
- data_type: 3
  data: 917.0
- data_type: 2
  data: 176.0
- data_type: 5
  data: 240.0
- data_type: 4
  data: 141.15276588141035
---
measurement_source_name: minimal_subscriber_with_topic_statistics
metrics_source: message_period
unit: ms
window_start:
  sec: 1626297589
  nanosec: 164521929
window_stop:
  sec: 1626297599
  nanosec: 164252629
statistics:
- data_type: 1
  data: 498.1297071129706
- data_type: 3
  data: 951.0
- data_type: 2
  data: 84.0
- data_type: 5
  data: 239.0
- data_type: 4
  data: 199.2434118131896
---
measurement_source_name: minimal_subscriber_with_topic_statistics
metrics_source: message_age
unit: ms
window_start:
  sec: 1626297599
  nanosec: 164252629
window_stop:
  sec: 1626297609
  nanosec: 164037083
statistics:
- data_type: 1
  data: 642.7423076923083
- data_type: 3
  data: 917.0
- data_type: 2
  data: 176.0
- data_type: 5
  data: 260.0
- data_type: 4
  data: 140.46573267602685
---
measurement_source_name: minimal_subscriber_with_topic_statistics
metrics_source: message_period
unit: ms
window_start:
  sec: 1626297599
  nanosec: 164252629
window_stop:
  sec: 1626297609
  nanosec: 164037083
statistics:
- data_type: 1
  data: 499.32432432432427
- data_type: 3
  data: 951.0
- data_type: 2
  data: 84.0
- data_type: 5
  data: 259.0
- data_type: 4
  data: 199.68864525904544
---
measurement_source_name: minimal_subscriber_with_topic_statistics
metrics_source: message_age
unit: ms
window_start:
  sec: 1626297609
  nanosec: 164037083
window_stop:
  sec: 1626297619
  nanosec: 163813017
statistics:
- data_type: 1
  data: 642.7357142857148
- data_type: 3
  data: 917.0
- data_type: 2
  data: 176.0
- data_type: 5
  data: 280.0
- data_type: 4
  data: 139.49795752290154
---
measurement_source_name: minimal_subscriber_with_topic_statistics
metrics_source: message_period
unit: ms
window_start:
  sec: 1626297609
  nanosec: 164037083
window_stop:
  sec: 1626297619
  nanosec: 163813017
statistics:
- data_type: 1
  data: 498.57347670250886
- data_type: 3
  data: 951.0
- data_type: 2
  data: 84.0
- data_type: 5
  data: 279.0
- data_type: 4
  data: 198.7640867055671
---
measurement_source_name: minimal_subscriber_with_topic_statistics
metrics_source: message_age
unit: ms
window_start:
  sec: 1626297619
  nanosec: 163813017
window_stop:
  sec: 1626297629
  nanosec: 163557748
statistics:
- data_type: 1
  data: 645.1033333333339
- data_type: 3
  data: 917.0
- data_type: 2
  data: 176.0
- data_type: 5
  data: 300.0
- data_type: 4
  data: 139.76480001138424
---
measurement_source_name: minimal_subscriber_with_topic_statistics
metrics_source: message_period
unit: ms
window_start:
  sec: 1626297619
  nanosec: 163813017
window_stop:
  sec: 1626297629
  nanosec: 163557748
statistics:
- data_type: 1
  data: 499.53511705685605
- data_type: 3
  data: 951.0
- data_type: 2
  data: 84.0
- data_type: 5
  data: 299.0
- data_type: 4
  data: 199.6823261470278
---

## TimeReference

ubuntu:~/hadabot_main/content/p11/hadabot_ws (master) $ ros2 topic echo /statistics/ping_time_reference 
1626297819.735569 [0]       ros2: using network interface ens3 (udp/10.0.0.134) selected arbitrarily from: ens3, docker0
measurement_source_name: minimal_subscriber_with_topic_statistics
metrics_source: message_age
unit: ms
window_start:
  sec: 1626297811
  nanosec: 105750685
window_stop:
  sec: 1626297821
  nanosec: 105616221
statistics:
- data_type: 1
  data: 632.7142857142858
- data_type: 3
  data: 782.0
- data_type: 2
  data: 478.0
- data_type: 5
  data: 21.0
- data_type: 4
  data: 92.29050555157876
---
measurement_source_name: minimal_subscriber_with_topic_statistics
metrics_source: message_period
unit: ms
window_start:
  sec: 1626297811
  nanosec: 105750685
window_stop:
  sec: 1626297821
  nanosec: 105616221
statistics:
- data_type: 1
  data: 494.45
- data_type: 3
  data: 779.0
- data_type: 2
  data: 409.0
- data_type: 5
  data: 20.0
- data_type: 4
  data: 135.32312256225836
---
measurement_source_name: minimal_subscriber_with_topic_statistics
metrics_source: message_age
unit: ms
window_start:
  sec: 1626297821
  nanosec: 105616221
window_stop:
  sec: 1626297831
  nanosec: 105359705
statistics:
- data_type: 1
  data: 637.7250000000001
- data_type: 3
  data: 910.0
- data_type: 2
  data: 446.0
- data_type: 5
  data: 40.0
- data_type: 4
  data: 109.43993501003187
---
measurement_source_name: minimal_subscriber_with_topic_statistics
metrics_source: message_period
unit: ms
window_start:
  sec: 1626297821
  nanosec: 105616221
window_stop:
  sec: 1626297831
  nanosec: 105359705
statistics:
- data_type: 1
  data: 505.3333333333333
- data_type: 3
  data: 885.0
- data_type: 2
  data: 342.0
- data_type: 5
  data: 39.0
- data_type: 4
  data: 158.79358207875313
---
measurement_source_name: minimal_subscriber_with_topic_statistics
metrics_source: message_age
unit: ms
window_start:
  sec: 1626297831
  nanosec: 105359705
window_stop:
  sec: 1626297841
  nanosec: 105114601
statistics:
- data_type: 1
  data: 648.1833333333335
- data_type: 3
  data: 956.0
- data_type: 2
  data: 446.0
- data_type: 5
  data: 60.0
- data_type: 4
  data: 117.26714397287739
---
measurement_source_name: minimal_subscriber_with_topic_statistics
metrics_source: message_period
unit: ms
window_start:
  sec: 1626297831
  nanosec: 105359705
window_stop:
  sec: 1626297841
  nanosec: 105114601
statistics:
- data_type: 1
  data: 500.4237288135594
- data_type: 3
  data: 931.0
- data_type: 2
  data: 331.0
- data_type: 5
  data: 59.0
- data_type: 4
  data: 162.33802392912938
---
measurement_source_name: minimal_subscriber_with_topic_statistics
metrics_source: message_age
unit: ms
window_start:
  sec: 1626297841
  nanosec: 105114601
window_stop:
  sec: 1626297851
  nanosec: 104869949
statistics:
- data_type: 1
  data: 630.9506172839508
- data_type: 3
  data: 956.0
- data_type: 2
  data: 242.0
- data_type: 5
  data: 81.0
- data_type: 4
  data: 126.519202781469
---
measurement_source_name: minimal_subscriber_with_topic_statistics
metrics_source: message_period
unit: ms
window_start:
  sec: 1626297841
  nanosec: 105114601
window_stop:
  sec: 1626297851
  nanosec: 104869949
statistics:
- data_type: 1
  data: 497.0625
- data_type: 3
  data: 931.0
- data_type: 2
  data: 94.0
- data_type: 5
  data: 80.0
- data_type: 4
  data: 168.4834816643756
---
measurement_source_name: minimal_subscriber_with_topic_statistics
metrics_source: message_age
unit: ms
window_start:
  sec: 1626297851
  nanosec: 104869949
window_stop:
  sec: 1626297861
  nanosec: 104622603
statistics:
- data_type: 1
  data: 631.8400000000003
- data_type: 3
  data: 956.0
- data_type: 2
  data: 242.0
- data_type: 5
  data: 100.0
- data_type: 4
  data: 121.30397520279377
---
measurement_source_name: minimal_subscriber_with_topic_statistics
metrics_source: message_period
unit: ms
window_start:
  sec: 1626297851
  nanosec: 104869949
window_stop:
  sec: 1626297861
  nanosec: 104622603
statistics:
- data_type: 1
  data: 500.13131313131316
- data_type: 3
  data: 931.0
- data_type: 2
  data: 94.0
- data_type: 5
  data: 99.0
- data_type: 4
  data: 164.38803849826172
---
measurement_source_name: minimal_subscriber_with_topic_statistics
metrics_source: message_age
unit: ms
window_start:
  sec: 1626297861
  nanosec: 104622603
window_stop:
  sec: 1626297871
  nanosec: 104375775
statistics:
- data_type: 1
  data: 633.5785123966946
- data_type: 3
  data: 956.0
- data_type: 2
  data: 242.0
- data_type: 5
  data: 121.0
- data_type: 4
  data: 121.13347273140012
---
measurement_source_name: minimal_subscriber_with_topic_statistics
metrics_source: message_period
unit: ms
window_start:
  sec: 1626297861
  nanosec: 104622603
window_stop:
  sec: 1626297871
  nanosec: 104375775
statistics:
- data_type: 1
  data: 498.15000000000003
- data_type: 3
  data: 931.0
- data_type: 2
  data: 94.0
- data_type: 5
  data: 120.0
- data_type: 4
  data: 164.03295654633956
---
measurement_source_name: minimal_subscriber_with_topic_statistics
metrics_source: message_age
unit: ms
window_start:
  sec: 1626297871
  nanosec: 104375775
window_stop:
  sec: 1626297881
  nanosec: 104133693
statistics:
- data_type: 1
  data: 635.9785714285717
- data_type: 3
  data: 956.0
- data_type: 2
  data: 242.0
- data_type: 5
  data: 140.0
- data_type: 4
  data: 121.67788083161815
---
measurement_source_name: minimal_subscriber_with_topic_statistics
metrics_source: message_period
unit: ms
window_start:
  sec: 1626297871
  nanosec: 104375775
window_stop:
  sec: 1626297881
  nanosec: 104133693
statistics:
- data_type: 1
  data: 497.9352517985611
- data_type: 3
  data: 931.0
- data_type: 2
  data: 94.0
- data_type: 5
  data: 139.0
- data_type: 4
  data: 164.79790146010967
---
measurement_source_name: minimal_subscriber_with_topic_statistics
metrics_source: message_age
unit: ms
window_start:
  sec: 1626297881
  nanosec: 104133693
window_stop:
  sec: 1626297891
  nanosec: 103887940
statistics:
- data_type: 1
  data: 636.8312500000003
- data_type: 3
  data: 956.0
- data_type: 2
  data: 242.0
- data_type: 5
  data: 160.0
- data_type: 4
  data: 124.59525582235261
---
measurement_source_name: minimal_subscriber_with_topic_statistics
metrics_source: message_period
unit: ms
window_start:
  sec: 1626297881
  nanosec: 104133693
window_stop:
  sec: 1626297891
  nanosec: 103887940
statistics:
- data_type: 1
  data: 499.4088050314466
- data_type: 3
  data: 931.0
- data_type: 2
  data: 94.0
- data_type: 5
  data: 159.0
- data_type: 4
  data: 171.73470156381777
---
measurement_source_name: minimal_subscriber_with_topic_statistics
metrics_source: message_age
unit: ms
window_start:
  sec: 1626297891
  nanosec: 103887940
window_stop:
  sec: 1626297901
  nanosec: 103641504
statistics:
- data_type: 1
  data: 633.4806629834258
- data_type: 3
  data: 956.0
- data_type: 2
  data: 242.0
- data_type: 5
  data: 181.0
- data_type: 4
  data: 128.48116533980482
---
measurement_source_name: minimal_subscriber_with_topic_statistics
metrics_source: message_period
unit: ms
window_start:
  sec: 1626297891
  nanosec: 103887940
window_stop:
  sec: 1626297901
  nanosec: 103641504
statistics:
- data_type: 1
  data: 497.49444444444447
- data_type: 3
  data: 931.0
- data_type: 2
  data: 84.0
- data_type: 5
  data: 180.0
- data_type: 4
  data: 174.94575340888522
---


## Wheel radps left 

ubuntu:~/hadabot_main/content/p11/hadabot_ws (master) $ ros2 topic echo /statistics/wheel_radps_left 
1626297715.854296 [0]       ros2: using network interface ens3 (udp/10.0.0.134) selected arbitrarily from: ens3, docker0
measurement_source_name: minimal_subscriber_with_topic_statistics
metrics_source: message_age
unit: ms
window_start:
  sec: 1626297706
  nanosec: 852837400
window_stop:
  sec: 1626297716
  nanosec: 852591434
statistics:
- data_type: 1
  data: .nan
- data_type: 3
  data: .nan
- data_type: 2
  data: .nan
- data_type: 5
  data: 0.0
- data_type: 4
  data: .nan
---
measurement_source_name: minimal_subscriber_with_topic_statistics
metrics_source: message_period
unit: ms
window_start:
  sec: 1626297706
  nanosec: 852837400
window_stop:
  sec: 1626297716
  nanosec: 852591434
statistics:
- data_type: 1
  data: 106.04324324324332
- data_type: 3
  data: 433.0
- data_type: 2
  data: 0.0
- data_type: 5
  data: 185.0
- data_type: 4
  data: 179.20493559283662
---
measurement_source_name: minimal_subscriber_with_topic_statistics
metrics_source: message_age
unit: ms
window_start:
  sec: 1626297716
  nanosec: 852591434
window_stop:
  sec: 1626297726
  nanosec: 852348566
statistics:
- data_type: 1
  data: .nan
- data_type: 3
  data: .nan
- data_type: 2
  data: .nan
- data_type: 5
  data: 0.0
- data_type: 4
  data: .nan
---
measurement_source_name: minimal_subscriber_with_topic_statistics
metrics_source: message_period
unit: ms
window_start:
  sec: 1626297716
  nanosec: 852591434
window_stop:
  sec: 1626297726
  nanosec: 852348566
statistics:
- data_type: 1
  data: 106.56159420289856
- data_type: 3
  data: 433.0
- data_type: 2
  data: 0.0
- data_type: 5
  data: 276.0
- data_type: 4
  data: 178.80740137327734
---
measurement_source_name: minimal_subscriber_with_topic_statistics
metrics_source: message_age
unit: ms
window_start:
  sec: 1626297726
  nanosec: 852348566
window_stop:
  sec: 1626297736
  nanosec: 852110782
statistics:
- data_type: 1
  data: .nan
- data_type: 3
  data: .nan
- data_type: 2
  data: .nan
- data_type: 5
  data: 0.0
- data_type: 4
  data: .nan
---
measurement_source_name: minimal_subscriber_with_topic_statistics
metrics_source: message_period
unit: ms
window_start:
  sec: 1626297726
  nanosec: 852348566
window_stop:
  sec: 1626297736
  nanosec: 852110782
statistics:
- data_type: 1
  data: 106.85714285714283
- data_type: 3
  data: 433.0
- data_type: 2
  data: 0.0
- data_type: 5
  data: 371.0
- data_type: 4
  data: 179.17772222489586
---
