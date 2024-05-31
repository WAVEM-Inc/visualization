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
            - [Compile Package](#3-2-1-2-compile-package)
        - [Build ROS Server](#3-2-2-build-ros-server)
          - [Compile Package](#3-2-2-1-compile-package)
        - [Build Web Server](#3-2-3-build-web-server)
          - [Build nodejs Server](#3-2-3-1-build-nodejs-server)
          - [Build react Client](#3-2-3-2-build-react-client)
  - [Launch](#4-launch)
    - [Modify MQTT broker's host adress (If Necessary)](#4-1-1-modify-mqtt-brokers-host-adress-if-necessary)
    - [Launch mosquitto broker](#4-1-2-launch-mosquitto-broker)
    - [Launch ROS Server](#4-2-launch-ros-server)
    - [Launch Web](#4-3-launch-web)


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
```

### 3-2. Build Project

### 3-2-1. Modify MQTT broker's host adress (If Necessary)
```bash
cd ~/RobotData/mqtt/
vi mqtt.json

{
  "host": "192.168.56.1",
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

### 3-2-2. Build ROS Server

### 3-2-2-1. Compile Package
```bash
cd ~/kec_ws/src/service/visualization/
colcon build --packages-select rmserver_kec
```

### 3-2-3. Build Web Server

### 3-2-3-1. Build nodejs Server
```bash
cd ~/kec_ws/src/service/visualization/rmserver/web/
npm run server_build
```

### 3-2-3-2. Build react Client
```bash
cd ~/kec_ws/src/service/visualization/rmserver/web/
npm run client_build
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

## 4-2. Launch ROS Server
```bash
source ~/kec_ws/src/total.bash
ros2 launch rmserver_kec rmserver_kec.launch.py
```

## 4-3. Launch Web
```bash
cd ~/kec_ws/src/service/visualization/rmserver/web
npm run launch
```