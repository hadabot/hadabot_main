from shared.utils import FOOTER


def main(argv=None):
    print(f"""
############################################################

1. Use ROS to see the messages that direct the Left motor to spin. In this web-bash terminal, type the following, ** one line at a time **:

     source /opt/ros/eloquent/setup.bash

     ros2 topic echo /hadabot/wheel_power_left

   You should see messages print when you slide the sliders from the Hadabot Dashboard. Ctrl-C to break out of the command.

2. Instead of using the web-GUI, use a ROS2 command line to turn the wheel.

     ros2 topic pub -1 /hadabot/wheel_power_left std_msgs/Float32 '{{ data: 0.8 }}'

   This will direct the Left motor to spin at 80% power.

     ros2 topic pub -1 /hadabot/wheel_power_left std_msgs/Float32 '{{ data: 0 }}'

   This will stop the Left motor.

{FOOTER}
############################################################
""")


if __name__ == "__main__":
    main()
