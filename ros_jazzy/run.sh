#!/usr/bin/env bash

docker run -it --rm \
  --net=host \
  -e DISPLAY=$DISPLAY \
  -e QT_QPA_PLATFORM=xcb \
  -e QT_X11_NO_MITSHM=1 \
  -e XDG_RUNTIME_DIR=/run/user/1000 \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  -v ~/Dissertation-RobotAutomation/ros_jazzy_ws:/home/ros/ros_ws \
  ros2-jazzy-dev
