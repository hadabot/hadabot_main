from shared.utils import FOOTER


def main(argv=None):
    print(f"""
1. Type the following, ** one line at a time **, into the terminal:

     source /opt/ros/foxy/setup.bash

     ros2 topic echo /chatter

  You should see the message from the terminal #1 being outputted.

  Ctrl-C to exit out.

Congrats, you have started your journey to ROS2 mastery!

{FOOTER}
""")


if __name__ == "__main__":
    main()
