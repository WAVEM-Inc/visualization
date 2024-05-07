import os;
import json;
import paho.mqtt.client as paho;
from rclpy.node import Node;
from rclpy.impl.rcutils_logger import RcutilsLogger;
from typing import Any;
from typing import Dict;
from rmserver_kec.application.mqtt import Client;

MQTT_PATH_EDIT_RESPONSE_TOPIC: str = "/rms/ktp/dummy/response/path/edit";

class PathProcessor:
    
    def __init__(self, node: Node, mqtt_client: Client) -> None:
        self.__node: Node = node;
        self.__log: RcutilsLogger = self.__node.get_logger();
        self.__mqtt_client: Client = mqtt_client;
        self.__current_path_file: str = self.__node.get_parameter("current_path_file").get_parameter_value().string_value;
        
        self.__path: Any = {};
        
    def mqtt_path_edit_request_cb(self, mqtt_client: paho.Client, mqtt_user_data: Dict, mqtt_message: paho.MQTTMessage) -> None:
        try:
            mqtt_topic: str = mqtt_message.topic;
            # mqtt_decoded_payload: str = mqtt_message.payload.decode();
            # mqtt_json: Any = json.loads(mqtt_message.payload);
            
            self.load_path();
            self.__mqtt_client.publish(topic=MQTT_PATH_EDIT_RESPONSE_TOPIC, payload=json.dumps(obj=self.__path), qos=0);
        except KeyError as ke:
            self.__log.error(f"Invalid JSON Key in MQTT {mqtt_topic} subscription callback: {ke}");
            return;

        except json.JSONDecodeError as jde:
            self.__log.error(f"Invalid JSON format in MQTT {mqtt_topic} subscription callback: {jde.msg}");
            return;

        except Exception as e:
            self.__log.error(f"Exception in MQTT {mqtt_topic} subscription callback: {e}");
            return;
    
    def load_path(self) -> None:
        home_directory: str = os.path.expanduser("~");
        map_path: str = f"{home_directory}/{self.__current_path_file}";
        
        try:
            with open(map_path, "r", encoding="utf-8") as f:
                self.__path = json.load(f);
                self.__log.info(f"Map Path: {map_path}");
        except FileNotFoundError as fne:
            self.__log.error(f"{fne}");
            return;
        
    
__all__: list[str] = ["PathProcessor"];