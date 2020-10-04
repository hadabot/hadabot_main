from shared.utils import FOOTER, CONTENT_FOLDER_PREFIX


def main(argv=None):
    print(f"""
############################################################
First read these instructions. Then do it!

You will only need one (1) terminal for this example. You should have a Hadabot ESP32 board loaded with the required firware files and connected to your network. The built-in LED on the ESP32 board should be lit which means connected.

You should also have the L9110 motor driver connected to 2 motors and the ESP32.

1. Open a new browser window. Go to:

     https://www.hadabot.com/tools/hadabot-dashboard.html

   The status panel indicate that it's connected to your local ROS2.

2. Let's spin the motors. Slide the sliders for the respective wheels on the Hadabot Dashboard.

3. Move on to the next step

     python3 {CONTENT_FOLDER_PREFIX}4/step_2.py

   Follow the instructions outputted.

{FOOTER}
############################################################
""")


if __name__ == "__main__":
    main()
