#! /bin/bash

sleep 0.5

ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "\
{\
pose: {\
  header: {\
    stamp: {\
      sec: 1607032613,\
      nanosec: 0,\
      },\
    frame_id: map\
    },\
  pose: {\
    position: {\
      x: 6.0,\
      y: 5.0,\
      z: 0.0\
      },\
    orientation: {\
      x: 0.0,\
      y: 0.0,\
      z: 0.0,\
      w: 1.0\
      }\
    }\
  }
}
"