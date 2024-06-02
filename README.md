# rmviz
- ROS & MQTT Visualization Web Application Package.

## Document
- [rmviz](#rmviz)
  - [Document](#document)
  - [Environment](#1-environment)
  - [SetUp Installation](#2-setup-installation)
    - [Prerequisites](#2-1-prerequisites)
      - [Install rosbridge_library](#2-1-1-install-rosbridge-library)
      - [Install mosquitto](#2-1-2-install-mosquitto)
      - [Install pip requirements](#2-1-3-install-pip-requirements)
      - [Install nodejs](#2-1-4-install-nodejs)
      - [Install npm](#2-1-5-install-npm)
      - [Install n & Install nodejs LTS Version](#2-1-6-install-n--install-nodejs-lts-version)
  - [Clone & Build Project](#3-clone--build-project)
    - [Clone Project](#3-1-clone-project)
    - [Build Project](#3-2-build-project)
        - [Modify MQTT Broker's Host Address(If Necessary)](#3-2-1-modify-mqtt-broker-host-adress-if-necessary)
        - [Build](#3-2-2-build)
          - [Install npm dependencies](#3-2-2-1-install-npm-dependencies)
          - [Build](#3-2-2-2-build)
  - [Launch](#4-launch)
    - [Launch mosquitto broker](#4-1-launch-mosquitto-broker)
      - [Modify MQTT broker's host adress (If Necessary)](#4-1-1-modify-mqtt-brokers-host-adress-if-necessary)
      - [Launch mosquitto broker](#4-1-2-launch-mosquitto-broker)
    - [Launch](#4-2-launch)


## 1. Environment
* <img src="https://img.shields.io/badge/ROS2 humble-22314E?style=for-the-badge&logo=ros&logoColor=white">
* <img src="https://img.shields.io/badge/ubuntu 22.04-E95420?style=for-the-badge&logo=ubuntu&logoColor=white">
* <img src="https://img.shields.io/badge/python 3.10.12-3670A0?style=for-the-badge&logo=python&logoColor=ffdd54">
* <img src="https://img.shields.io/badge/CMake-064F8C.svg?style=for-the-badge&logo=cmake&logoColor=white">
* <img src="https://img.shields.io/badge/mqtt-660066.svg?style=for-the-badge&logo=mqtt&logoColor=white">
* <img src="https://img.shields.io/badge/eclipse mosquitto-3C5280.svg?style=for-the-badge&logo=eclipse mosquitto&logoColor=white">
* ![TypeScript](https://img.shields.io/badge/typescript-3178C6.svg?style=for-the-badge&logo=typescript&logoColor=white)
* ![NodeJS](https://img.shields.io/badge/node.js-6DA55F?style=for-the-badge&logo=node.js&logoColor=white)
* ![NPM](https://img.shields.io/badge/npm-CB3837?style=for-the-badge&logo=npm&logoColor=white)
* ![React](https://img.shields.io/badge/react-%2320232a.svg?style=for-the-badge&logo=react&logoColor=%2361DAFB)
* ![HTML5](https://img.shields.io/badge/html5-%23E34F26.svg?style=for-the-badge&logo=html5&logoColor=white)
* ![CSS3](https://img.shields.io/badge/css3-%231572B6.svg?style=for-the-badge&logo=css3&logoColor=white)
* <img src="https://img.shields.io/badge/google Maps-4285F4.svg?style=for-the-badge&logo=google-maps&logoColor=red">

## 2. SetUp Installation

### 2-1. Prerequisites

Before installing, please ensure the following software is installed and configured on your system:

- [ubuntu](https://ubuntu.com/) version required 20.04 - **INSTALL [ubuntu 22.04](https://ubuntu.com/)**

- [ROS2](https://index.ros.org/doc/ros2/Installation/) version required humble-hawksbill -
  **INSTALL [ROS2 humble-hawksbill](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html)**

- [rosbridge_suite](https://github.com/RobotWebTools/rosbridge_suite) version required humble-hawksbill -
  **INSTALLL [rosbridge_suite](https://github.com/RobotWebTools/rosbridge_suite)**

- [python3](https://www.python.org/downloads/) version required over than 3.0 - 
  **INSTALL [python3](https://www.python.org/downloads/)**

### 2-1-1. Install rosbridge-library
```bash
sudo apt-get install ros-humble-rosbridge-library
```

### 2-1-2. Install mosquitto
```bash
sudo apt-get install mosquitto*
```

### 2-1-3. Install pip requirements
```bash
cd ~/kec_ws/src/rms/ktp_dummy_interface/
pip install -r pip_requirement.txt
```

### 2-1-4. Install nodejs
```bash
sudo apt install nodejs
```

### 2-1-5. Install npm
```bash
sudo apt install npm
```

### 2-1-6. Install n & Install nodejs LTS Version
```bash
sudo npm i -g n
sudo n lts

# check nodejs version
nodejs -v # 12.22.9
node -v # 20.12.2
npm -v # 10.5.0
```

## 3. Clone & Build Project

### 3-1. Clone Project
```bash
cd ~/kec_ws/src/service/
git clone -b KEC/Web/Humble/Develop https://github.com/WAVEM-Inc/visualization.git
mv ./visualization/ ./rmviz
```

### 3-2. Build Project

### 3-2-1. Modify MQTT broker's host adress (If Necessary)
```bash
cd ~/RobotData/mqtt/
vi mqtt.json

{
  "host": "${your IP address}",
  "port": 8883,
  "protocol": "ws",
  "type": "websockets",
  "path": "/ws",
  "client_name": "wavemrmserver",
  "password":"22087",
  "user_name":"wavemrmserver"
}

# :wq
```

### 3-2-2. Build 

### 3-2-2-1. Install npm dependencies
```bash
cd ~/kec_ws/src/service/rmviz/rmserver/http/
npm i

cd ~/kec_ws/src/service/rmviz/rmviz/
npm i
```

### 3-2-2-2. Build
```bash
cd ~/kec_ws/src/service/rmviz/rmserver/http/
npm run build
```

## 4. Launch

## 4-1. Launch mosquitto broker

### 4-1-1. Modify MQTT broker's host adress (If Necessary)
```bash
cd ~/RobotData/mqtt
vi ./mosquitto.conf

listener 1883 ${your IP address}
protocol mqtt
allow_anonymous true

listener 8883 ${your IP address}
protocol websockets
allow_anonymous true

# :wq
```
### 4-1-2. Launch mosquitto broker
```bash
cd ~/RobotData/mqtt/
sudo chmod +x ./mosquitto.sh
./mosquitto.sh
```

## 4-2. Launch 
```bash
# Check ./.bashrc alias
vi ./.bashrc

alias rmviz="cd ~/kec_ws/src/service/rmviz/rmserver/http && npm run launch"

# Launch
rmviz

# Launch Success Example
${your user name}@${your PC name}:~$ rmviz 

> rmvserver_web@1.0.0 launch
> sudo chmod +x ./script/launch.sh && ./script/launch.sh


> rmvserver_web@1.0.0 start
> node dist/app.js

[INFO] [launch]: All log files can be found below /home/reidlo/.ros/log/2024-06-01-11-33-32-142801-reidlo-VirtualBox-14491
[INFO] [launch]: Default logging verbosity is set to INFO
[INFO] [rmserver_rqtt-1]: process started with pid [14511]
[rmserver_http] [INFO] dirname : /home/reidlo/kec_ws/src/service/rmviz/rmserver/http/dist, client_path : /home/reidlo/kec_ws/src/service/rmviz/rmviz/build
[rmserver_http] [INFO] Server is running at port <3000>

[rmserver_rqtt-1] [INFO] [1717209212.580991721] [rmserver_rqtt]: rmserver_rqtt created
[rmserver_rqtt-1] [INFO] [1717209212.581950881] [rmserver_rqtt]: rmserver_rqtt Declaring key : [current_mqtt_file], value : []
[rmserver_rqtt-1] [INFO] [1717209212.583393931] [rmserver_rqtt]: rmserver_rqtt Declaring key : [current_map_config_file], value : []
[rmserver_rqtt-1] [INFO] [1717209212.584949060] [rmserver_rqtt]: rmserver_rqtt Declaring key : [current_urdf_file], value : []
[rmserver_rqtt-1] [INFO] [1717209212.586514493] [rmserver_rqtt]: MQTT Path: /home/reidlo/RobotData/mqtt/mqtt.json
[rmserver_rqtt-1] [INFO] [1717209212.587614086] [rmserver_rqtt]: MQTT Connect
[rmserver_rqtt-1] host : 192.168.56.1
[rmserver_rqtt-1] port : 8883
[rmserver_rqtt-1] type : websockets
[rmserver_rqtt-1] 
[rmserver_rqtt-1] [INFO] [1717209212.748278463] [rmserver_rqtt]: Map Config Path: /home/reidlo/RobotData/maps/kecd_path/config/config.ini
[rmserver_rqtt-1] [INFO] [1717209212.748982600] [rmserver_rqtt]: Map Path : /home/reidlo/RobotData/maps/kecd_path/KEC_ROUTE_20240530_4.dat
[rmserver_rqtt-1] [INFO] [1717209212.752249402] [rmserver_rqtt]: MQTT granted subscription
[rmserver_rqtt-1] 	topic : /rmviz/request/detected_object
[rmserver_rqtt-1] 	qos : 0
[rmserver_rqtt-1] [INFO] [1717209212.753416487] [rmserver_rqtt]: MQTT Succeeded to Connect
[rmserver_rqtt-1] [INFO] [1717209212.755142482] [rmserver_rqtt]: MQTT granted subscription
[rmserver_rqtt-1] 	topic : /rmviz/request/obstacle/status
[rmserver_rqtt-1] 	qos : 0
[rmserver_rqtt-1] [INFO] [1717209212.757894571] [rmserver_rqtt]: MQTT granted subscription
[rmserver_rqtt-1] 	topic : /rmviz/request/obstacle/cooperative
[rmserver_rqtt-1] 	qos : 0
[rmserver_rqtt-1] [INFO] [1717209212.760173616] [rmserver_rqtt]: MQTT granted subscription
[rmserver_rqtt-1] 	topic : /rmviz/request/can/emergency
[rmserver_rqtt-1] 	qos : 0
[rmserver_rqtt-1] [INFO] [1717209212.762000504] [rmserver_rqtt]: MQTT granted subscription
[rmserver_rqtt-1] 	topic : /rmviz/request/route_to_pose
[rmserver_rqtt-1] 	qos : 0
[rmserver_rqtt-1] [INFO] [1717209212.764982774] [rmserver_rqtt]: MQTT granted subscription
[rmserver_rqtt-1] 	topic : /rmviz/request/goal/cancel
[rmserver_rqtt-1] 	qos : 0
[rmserver_rqtt-1] [INFO] [1717209212.767568581] [rmserver_rqtt]: MQTT granted subscription
[rmserver_rqtt-1] 	topic : /rmviz/request/can/init
[rmserver_rqtt-1] 	qos : 0
[rmserver_rqtt-1] [INFO] [1717209212.769811740] [rmserver_rqtt]: MQTT granted subscription
[rmserver_rqtt-1] 	topic : /rmviz/request/heartbeat
[rmserver_rqtt-1] 	qos : 0
[rmserver_rqtt-1] [INFO] [1717209212.772274004] [rmserver_rqtt]: MQTT granted subscription
[rmserver_rqtt-1] 	topic : /rmviz/request/task
[rmserver_rqtt-1] 	qos : 0
[rmserver_rqtt-1] [INFO] [1717209212.774253026] [rmserver_rqtt]: MQTT granted subscription
[rmserver_rqtt-1] 	topic : /rmviz/request/path/renew
[rmserver_rqtt-1] 	qos : 0
[rmserver_rqtt-1] [INFO] [1717209212.776730083] [rmserver_rqtt]: MQTT granted subscription
[rmserver_rqtt-1] 	topic : /rmviz/request/urdf
[rmserver_rqtt-1] 	qos : 0
[rmserver_rqtt-1] [INFO] [1717209212.779110372] [rmserver_rqtt]: MQTT granted subscription
[rmserver_rqtt-1] 	topic : /rmviz/request/gps/init
[rmserver_rqtt-1] 	qos : 0
[rmserver_rqtt-1] [INFO] [1717209212.781029708] [rmserver_rqtt]: MQTT granted subscription
[rmserver_rqtt-1] 	topic : /rmviz/request/path/select
[rmserver_rqtt-1] 	qos : 0
[rmserver_rqtt-1] [INFO] [1717209212.787345563] [rmserver_rqtt]: package_shared_directory : /home/reidlo/kec_ws/src/service/rmviz/rmserver/ros/install/rmserver_rqtt/share/rmserver_rqtt
[rmserver_rqtt-1] [INFO] [1717209212.788415946] [rmserver_rqtt]: DETECTECD JSON PATH : /home/reidlo/kec_ws/src/service/rmviz/rmserver/ros/install/rmserver_rqtt/share/rmserver_rqtt/json/detected_object.json

```