#!/bin/bash
npm run server_build
npm run client_build

if ! grep -qxF 'alias rmviz="cd ~/kec_ws/src/service/rmviz/rmserver/http && npm run launch"' ~/.bashrc; then
    echo 'alias rmviz="cd ~/kec_ws/src/service/rmviz/rmserver/http && npm run launch"' >> ~/.bashrc
fi

if ! grep -qxF 'alias mqtt_broker="sudo ~/RobotData/mqtt/mosquitto.sh"' ~/.bashrc; then
    echo 'alias mqtt_broker="sudo ~/RobotData/mqtt/mosquitto.sh"' >> ~/.bashrc
fi

source ~/.bashrc
