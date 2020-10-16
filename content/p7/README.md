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

3. Once build completes, in the terminal type:

```
source install/setup.bash
ros2 run hadabot_driver hadabot_controller
```

4. Click the link to launch the [Hadabot browser-based unicycle teleop controller](https://www.hadabot.com/tools/teleop.html).

1. Put the Hadabot on the ground and use the teleop controller to drive it around.

-----

## Step through the twist_cb() function with the debugger

The twist_cb(...) function implements the unicycle to differential drive equations.

1. Set a breakpoint somewhere in the _hadabot_controller.cpp::HadabotController::twist_cb()_ function.
    1. In VSCode, just click to the left of the line number to set a breakpoint at that line. A red dot will appear.
1. Hit F5 (or click upper left menu icon -> Run -> Start Debugging)
1. Upon first run, in the __DEBUG CONSOLE__, you'll see the hadabot_controller "has exited with code 0".
    1. Click the __TERMINAL__ sub-window.
    1. Confirm on the terminal name on the right of the __TERMINAL__ top bar says "cppdbg: hadabot_controller". You should also see the error output "...cannot open shared object file: No such file or directory".
    1. In this "cppdbg: hadabot_controller" terminal type:

```
source hadabot_ws/install/setup.bash
```

4. Hit F5 to start debugging again. This time, the ROS2 environment will be set up with linker path locations, etc..
