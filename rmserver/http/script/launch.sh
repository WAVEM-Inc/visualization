#!/bin/bash
source ~/kec_ws/src/service/rmviz/rmserver/rqtt/install/local_setup.bash
ros2 launch rmserver_rqtt rmserver_rqtt.launch.py &
npm run start
