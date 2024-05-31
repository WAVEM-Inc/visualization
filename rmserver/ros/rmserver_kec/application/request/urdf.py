import os;
import json;
import xmltodict;
import paho.mqtt.client as paho;
from rclpy.node import Node;
from rclpy.impl.rcutils_logger import RcutilsLogger;
from rmserver_kec.application.mqtt import Client;
from typing import Any;
from typing import Dict;


MQTT_URDF_RESPONSE_TOPIC: str = "/rmviz/response/urdf";


class URDFProcessor:
    
    def __init__(self, node: Node, mqtt_client: Client) -> None:
        self.__node: Node = node;
        self.__log: RcutilsLogger = self.__node.get_logger();
        self.__mqtt_client: Client = mqtt_client;
        self.__current_urdf_file: str = self.__node.get_parameter("current_urdf_file").get_parameter_value().string_value;
        
    def load_urdf(self) -> Any:
        home_directory: str = os.path.expanduser("~");
        urdf_path: str = f"{home_directory}/{self.__current_urdf_file}";
        urdf: Any = {};
        
        try:
            with open(urdf_path, "r", encoding="utf-8") as f:
                # urdf = json.loads(json.dumps(xmltodict.parse(f.read())));
                urdf = f.read();
                self.__log.info(f"URDF Path: {urdf_path}");
            return urdf;
        except FileNotFoundError as fne:
            self.__log.error(f"{fne}");
            return None;
        
    def mqtt_urdf_cb(self, mqtt_client: paho.Client, mqtt_user_data: Dict, mqtt_message: paho.MQTTMessage) -> None:
        try:
            mqtt_topic: str = mqtt_message.topic;
            mqtt_json: Any = json.loads(mqtt_message.payload);
            self.__log.info(f"{mqtt_topic}\n{json.dumps(mqtt_json, indent=4, ensure_ascii=False)}");
            
            urdf: Any = self.load_urdf();
            self.__log.info(f"{mqtt_topic} urdf : {urdf}");
            self.__mqtt_client.publish(topic=MQTT_URDF_RESPONSE_TOPIC, payload=urdf, qos=0);
            
        except KeyError as ke:
            self.__log.error(f"Invalid JSON Key in MQTT {mqtt_topic} subscription callback: {ke}");
            return;

        except json.JSONDecodeError as jde:
            self.__log.error(f"Invalid JSON format in MQTT {mqtt_topic} subscription callback: {jde.msg}");
            return;

        except Exception as e:
            self.__log.error(f"Exception in MQTT {mqtt_topic} subscription callback: {e}");
            return;

__all__: list[str] = ["URDFProcessor"];