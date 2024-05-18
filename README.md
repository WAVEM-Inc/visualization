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
        - [Build Server](#3-2-1-build-server)
            - [Modify MQTT Broker Host Address(If Necessary)](#3-2-1-1-modify-mqtt-broker-host-adress-if-necessary)
            - [Compile Package](#3-2-1-2-compile-package)
        - [Build Client](#3-2-2-build-client)
            - [Modify MQTT Broker Host Address(If Necessary)](#3-2-2-1-modify-mqtt-broker-host-adress-if-necessary)
            - [Install NPM Dependencies & Compile Package](#3-2-2-2-install-npm-dependencies--compile-package)
  - [Launch Check](#4-build-check)


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

### 3-2-1. Build Server

### 3-2-1-1. Modify MQTT broker host adress (If Necessary)
```bash
cd ~/kec_ws/src/service/visualization/rmserver/kec
vi config/mqtt_config.yaml

/rmserver_kec:
  ros__parameters:
    host: ${Your Current PC IP Address}
    port: 8883
    client_id: "wavem1234"
    client_keep_alive: 60
    user_name: "wavem"
    password: "1234"
    type: "websockets"
    path: "/ws"

# :wq
```

### 3-2-1-2. Compile Package
```bash
cd ~/kec_ws/src/service/visualization/
colcon build --packages-select rmserver_kec
```

### 3-2-2. Build Client

### 3-2-2-1. Modify MQTT broker host adress (If Necessary)
```bash
cd ~/kec_ws/src/service/visualization/rmviz

vi src/assets/config/mqtt.json

{
    "host": ${Your Current PC IP Address},
    "port": 8883,
    "protocol": "ws"
}

# :wq
```

### 3-2-2-2. Install npm dependencies & Compile Package
```bash
cd ~/kec_ws/src/service/visualization/rmviz
npm i
sudo npm i -g serve
npm run build
```

## 4. Launch

## 4-1. Launch mosquitto
```bash
cd ~/kec_ws/src/service/visualization/
sudo chmod +x ./mosquitto.sh
./mosquitto.sh
```

## 4-2. Launch Server
```bash
source clion_setup.bash
ros2 launch rmserver_kec rmserver_kec.launch.py
```

## 4-3. Launch Client
```bash
cd ~/kec_ws/src/service/visualization/rmviz
serve -s build -l 3000
```