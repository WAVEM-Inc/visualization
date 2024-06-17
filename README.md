# rmviz
- ROS & MQTT Visualization Web Application Package.

## Document
- [rmviz](#rmviz)
  - [Document](#document)
  - [Environment](#1-environment)
  - [SetUp Installation](#2-setup-installation)
    - [Prerequisites](#2-1-prerequisites)
      - [Install mosquitto](#2-1-1-install-mosquitto)
        - [Modify mosquitto configuration](#2-1-1-1-modify-mosquitto-configuration)
      - [Install nodejs](#2-1-2-install-nodejs)
      - [Install npm](#2-1-3-install-npm)
      - [Install n & Install nodejs LTS Version](#2-1-4-install-n--install-nodejs-lts-version)
  - [Clone & Build Project](#3-clone--build-project)
    - [Clone Project](#3-1-clone-project)
    - [Build Project](#3-2-build-project)
        - [Modify MQTT Broker's Host Address(If Necessary)](#3-2-1-modify-mqtt-brokers-host-address-if-necessary)
        - [Build](#3-2-2-build)
  - [Launch](#4-launch)


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

### 2-1-1. Install mosquitto
```bash
sudo apt-get install mosquitto*
```

### 2-1-1-1. Modify mosquitto configuration
```bash
sudo vi /lib/systemd/system/mosquitto.service

---

[Unit]
Description=Mosquitto MQTT Broker
Documentation=man:mosquitto.conf(5) man:mosquitto(8)
After=network.target
Wants=network.target

[Service]
Type=notify
NotifyAccess=main
ExecStart=/usr/sbin/mosquitto -v -c /home/reidlo/RobotData/mqtt/mosquitto.conf # modification, Before) /etc/mosquitto/mosquitto.conf
ExecReload=/bin/kill -HUP $MAINPID
Restart=on-failure
ExecStartPre=/bin/mkdir -m 740 -p /var/log/mosquitto
ExecStartPre=/bin/chown mosquitto /var/log/mosquitto
ExecStartPre=/bin/mkdir -m 740 -p /run/mosquitto
ExecStartPre=/bin/chown mosquitto /run/mosquitto

[Install]
WantedBy=multi-user.target

---

sudo systemctl daemon-reload
sudo systemctl restart mosquitto
sudo systemctl enable mosquitto
sudo systemctl status mosquitto

---

● mosquitto.service - Mosquitto MQTT Broker
     Loaded: loaded (/lib/systemd/system/mosquitto.service; enabled; vendor preset: enabled)
     Active: active (running) since Mon 2024-06-17 10:38:01 KST; 31s ago
       Docs: man:mosquitto.conf(5)
             man:mosquitto(8)
   Main PID: 15536 (mosquitto)
      Tasks: 1 (limit: 18992)
     Memory: 1.9M
        CPU: 27ms
     CGroup: /system.slice/mosquitto.service
             └─15536 /usr/sbin/mosquitto -v -c /home/reidlo/RobotData/mqtt/mosquitto.conf

--- 
```

### 2-1-2. Install nodejs
```bash
sudo apt install nodejs
```

### 2-1-3. Install npm
```bash
sudo apt install npm
```

### 2-1-4. Install n & Install nodejs LTS Version
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

### 3-2-1. Modify MQTT broker's host address (If Necessary)
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

### 3-2-2. Build 
```bash
cd ~/kec_ws/src/service/rmviz/rmserver/
npm run build
```

## 4. Launch
```bash
# Check ./.bashrc alias
vi ./.bashrc

alias rmviz="cd ~/kec_ws/src/service/rmviz/rmserver/ && npm run launch"

# Launch
rmviz
```
