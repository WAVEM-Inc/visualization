sudo systemctl stop mosquitto.service

mosquitto --verbose --config-file ./virtual_mosquitto.conf
