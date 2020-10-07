# Convert unicycle to differential drive commands

## Compile and run the code

Follow these steps to compile the source code

1. Open a terminal by clicking on the upper left menu bar icon -> Terminal -> New Terminal

1. In the terminal, type: 

```
source /opt/ros/foxy/setup.bash
cd hadabot_ws
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Debug
```

3. Once the build completes, we need to open up a browser-based VNC window to kick off the application. Open a browser tab to go to http://localhost:9124/

### In the __browser-based VNC__ tab:

1. Kick off a bash terminal - Left-click the lower-left "chevron-like "system icon -> System Tools -> LXTerminal

1. In the new LXTerminal bash terminal:
    1. `$ cd hadabot_main/content/p8/hadabot_ws/`
    1. `source install/setup.bash`
    1. `$ cd launch`
    1. `$ ros2 launch hadabot_nav2_launch.py`

1. Two application windows will emerge - rviz and turtlesim. Make sure you can see both windows.

1. In the rviz application:
    1. Click on "Navigation2 Goal" button
    1. Click somewhere in the white patch, drag towards to direction, you should see a red arrow appear. That is the goal pose - position and orientation which Navigation2 will drive our turtlesim towards.

As watch the turtlesim move towards the goal pose you specified!

-----


