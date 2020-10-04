from shared.utils import CONTENT_FOLDER_PREFIX, FOOTER


def main(argv=None):
    print(f"""
First read these instructions. Then do it!

1. In the vs-studio, start a new terminal.

2. In this terminal term #1, type the following, ** one line at a time **, into the bash terminal:

     source /opt/ros/foxy/setup.bash

     ros2 topic pub /chatter std_msgs/String '{{data: "Hadabot GO!"}}'

   You should see messages being outputted.

3. In the terminal term #2 you opened from steps 1, type:

     python3 {CONTENT_FOLDER_PREFIX}1/rd_2.py

   Follow the instructions outputted.

{FOOTER}
""")


if __name__ == "__main__":
    main()
