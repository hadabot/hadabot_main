def main(argv=None):
    print("""
First read these instructions. Then do it!

1. Click on, then copy via Ctrl-C the URL location of this webpage (ie 'https://localhost:9000/#/containers/<<ROS2_CONSOLE_CONTAINER_ID>>/exec')

2. Open another web browser tab.

3. In the new tab, paste the URL location you copied in step 1 to start a new Portainer web-bash terminal. Click "Connect". You should have 2 tabs open with running bash terminals in both tabs.

4. In this web-base terminal tab #1, type the following, ** one line at a time **, into the bash terminal:

     source /opt/ros/eloquent/setup.bash

     ros2 topic pub /chatter std_msgs/String '{data: "ROSdroid GO!"}'

   You should see messages being outputted.

5. In the web-bash terminal tab #2 you opened from steps 1-3, type:

     python3 /content/1/rd_2.py

   Follow the instructions outputted.

----

* Read the ROSdroid blog - blog.rosdroid.com - to understand what you are doing.

""")


if __name__ == "__main__":
    main()
