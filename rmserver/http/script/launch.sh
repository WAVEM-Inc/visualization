#!/bin/bash
source ~/kec_ws/src/service/rmviz/rmserver/ros/install/local_setup.bash
ros2 launch rmserver_kec rmserver_kec.launch.py &
npm run start
