from shared.utils import FOOTER


def main(argv=None):
    print(f"""
############################################################

1. Go back to the Hadabot Dashboard web page.

2. Move the LEFT wheel's power slider up.

3. Use ROS to echo back the wheel rotational velocity. In this web-bash terminal, type the following, ** one line at a time **:

     source /opt/ros/eloquent/setup.bash

     ros2 topic echo /hadabot/wheel_radps_left

   See the velocity echo'd out in the web-bash terminal.

   Ctrl-C to break out of the command.

{FOOTER}
############################################################
""")


if __name__ == "__main__":
    main()
