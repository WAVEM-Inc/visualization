import json;
import paho.mqtt.client as paho;
from datetime import datetime;
from rclpy.node import Node;
from rclpy.impl.rcutils_logger import RcutilsLogger;
from rosbridge_library.internal import message_conversion;
from typing import Dict;
from typing import Any;
from rmserver_rqtt.application.mqtt import Client;

MQTT_HEARTBEAT_RESPONSE_TOPIC: str = "/rmviz/response/heartbeat";


class HeartBeatProcessor:
    
    def __init__(self, node: Node, mqtt_client: Client) -> None:
        self.__node: Node = node;
        self.__log: RcutilsLogger = self.__node.get_logger();
        self.__mqtt_client: Client = mqtt_client;
        
    def mqtt_heart_beat_subscription_cb(self, mqtt_client: paho.Client, mqtt_user_data: Dict, mqtt_message: paho.MQTTMessage) -> None:
        try:
            mqtt_topic: str = mqtt_message.topic;
            mqtt_decoded_payload: str = mqtt_message.payload.decode();
            mqtt_json: Any = json.loads(mqtt_message.payload);
            
            requested_time: str = mqtt_json["request_time"];
            response_time: str = self.get_current_time();
            ping_differ: int = abs(int(float(requested_time) - float(response_time)));
            
            payload: str = json.dumps({"ping_differ": ping_differ});
            self.__mqtt_client.publish(topic=MQTT_HEARTBEAT_RESPONSE_TOPIC, payload=payload, qos=0);
        except KeyError as ke:
            self.__log.error(f"Invalid JSON Key in MQTT {mqtt_topic} subscription callback: {ke}");
            return;

        except json.JSONDecodeError as jde:
            self.__log.error(f"Invalid JSON format in MQTT {mqtt_topic} subscription callback: {jde.msg}");
            return;

        except message_conversion.NonexistentFieldException as nefe:
            self.__log.error(f"{mqtt_topic} : {nefe}");
            return;

        except Exception as e:
            self.__log.error(f"Exception in MQTT {mqtt_topic} subscription callback: {e}");
            return;

    def get_current_time(self) -> str:
        now: datetime = datetime.now();
        
        formatted_time: str = now.strftime("%y%m%d%H%M%S%f")[:-3];
        
        return formatted_time;
    

__all__: list[str] = ["HeartBeatProcessor"];