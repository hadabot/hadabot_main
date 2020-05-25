# Compute Hadabot Differential Drive Odometry from Wheel Rotational Velocity

## Compile and run the code

Follow these steps to compile the source code

1. Open a terminal by clicking on the upper left menu bar icon -> Terminal -> New Terminal

1. In the terminal, type: 

```
source /opt/ros/eloquent/setup.bash
cd hadabot_ws
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Debug
```

3. Once build completes, in the terminal type:

```
source install/setup.bash
ros2 run hadabot_driver hadabot_odom
```

4. Start a 2nd terminal next to your current terminal by clicking the side-by-side panel icon to the left of the __TERMINAL__ sub-menu bar. If you hover over the icon, it should say "Split Terminal".

1. In the new terminal, type the following command to see the initial odometry.

```
source install/setup.bash
ros2 topic echo /hadabot/odom
```

### Use pre-saved wheel rotational velocity data to update odometry

Let's play back some wheel rotational velocity messages we pre-saved as [ROS Bags](http://wiki.ros.org/Bags) so the odometry gets updated.

1. "Split Terminal" again to create a new terminal. __You should now have 3 terminal windows opened side-by-side__.
    1. __PRO TIP__: The horizontal layout may be getting crowded. Right-click in the empty space to the right of the "DEBUG CONSOLE" in the TERMINAL menu bar. A drop-down menu will appear. Select "Move Panel Right" to get a vertical layout.

1. In the new, third, terminal, type:

```
source install/setup.bash
cd ~/hadabot_main/content/p6
ros2 bag play rosbag2_wheel_rotational_velocity_data
```

2. At this point, you should start seeing updated pose and velocity (Twist) data being echo'd back in your 2nd "topic echo" terminal window.

1. Ctrl-C the running hadabot_odom and "topic echo" commands to stop their executions.

-----

## Step through the update_odometry() function with the debugger

1. Set a breakpoint somewhere in the _hadabot_odom.cpp::HadabotDriver::update_odometry()_ function.
    1. In VSCode, just click to the left of the line number to set a breakpoint at that line. A red dot will appear.
1. Hit F5 (or click upper left menu icon -> Run -> Start Debugging)
1. Upon first run, in the __DEBUG CONSOLE__, you'll see the hadabot_odom "has exited with code 0".
    1. Click the __TERMINAL__ sub-window.
    1. Confirm on the terminal name on the right of the __TERMINAL__ top bar says "cppdbg: hadabot_odom". You should also see the error output "...cannot open shared object file: No such file or directory".
    1. In this "cppdbg: hadabot_odom" terminal type:

```
source hadabot_ws/install/setup.bash
```

4. Hit F5 to start debugging again. This time, the ROS2 environment will be set up with linker path locations, etc..
