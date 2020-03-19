from shared.utils import FOOTER


def main(argv=None):
    print(f"""
############################################################
First read these instructions. Then do it!

You will only need one (1) web-bash terminal for this example. You should have a Hadabot ESP32 board loaded with the required firware files and connected to your network. The built-in LED on the ESP32 board should be lit which means connected.

1. Open a new browser window. Go to:

     https://www.hadabot.com/tools/teleop.html

   The status panel indicate that it's connected to your local ROS2.

2. Let's see the ESP32 respond to a ROS2 message being published. In the teleop.html browser window, click any of the buttons. Each time you click and release, you should see the ESP32's on-board LED flash.

3. Our ESP32 can also publish ROS2 messages. In this web-bash terminal, type the following, ** one line at a time **:

     source /opt/ros/eloquent/setup.bash

     ros2 topic echo /hadabot/log/info

   You should see a heartbeat message from the Hadabot every 10 seconds or so.

{FOOTER}
############################################################
""")


if __name__ == "__main__":
    main()
