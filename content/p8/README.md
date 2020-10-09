# Run the Navigation2, tf2, turtlesim example

## Compile and run the code

Follow these steps to compile the source code:

1. Open a terminal by clicking on the upper left menu bar icon -> Terminal -> New Terminal.

1. In the terminal, type: 

```
source /opt/ros/foxy/setup.bash
cd hadabot_ws
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Debug
```

3. Once the build completes, we need to open up a browser-based VNC window to kick off the application. Open a browser-based VNC environment via http://localhost:9124/

## In the __browser-based VNC__ tab:

1. Kick off a bash terminal:
    1. Left-click the lower-left "chevron-like" system icon -> System Tools -> LXTerminal.

1. In the new LXTerminal bash terminal:
    1. `$ cd hadabot_main/content/p8/hadabot_ws/`
    1. `$ source install/setup.bash`
    1. `$ cd launch`
    1. `$ ros2 launch hadabot_nav2_launch.py`

1. Two application windows will emerge - rviz and turtlesim. Make sure you can see both windows.

1. In the rviz application:
    1. Click on "Navigation2 Goal" button (in the top main menu bar).
    1. Click-n-hold somewhere in the white patch, drag to specify a direction, you should see a large green arrow appear. That is the goal pose - position and orientation which Navigation2 will drive our turtlesim towards.

In the turtlesim application window, you will see the turtlesim move towards the goal pose you specified!

------

## Want to try to implement hadabot_tf2_broadcaster yourself? 

1. From the file explorer menu to the left, open p8/hadabot_ws/src/hadabot_tf2/src/hadabot_tf2_broadcaster_diy.cpp 

1. Fill out the implementation for `void pose_callback(...)`.

1. Compile your code in your VSCode terminal

     1. Open a terminal by clicking on the upper left menu bar icon -> Terminal -> New Terminal.

     1. In the terminal, type: 

```
source /opt/ros/foxy/setup.bash
cd hadabot_ws
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Debug
```

1. Execute your code via the same instructions above with once exception - replace 

```
$ ros2 launch hadabot_nav2_launch.py
``` 

with 

```
$ ros2 launch hadabot_nav2_launch_diy.py
```
