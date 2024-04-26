import json;
import paho.mqtt.client as paho;
from rclpy.node import Node;
from rclpy.impl.rcutils_logger import RcutilsLogger;
from rmserver_kec.application.mqtt import Client;

class HeartBeatProcessor:
    
    def __init__(self, node: Node, mqtt_client: Client) -> None:
        self.__node: Node = node;
        self.__log: RcutilsLogger = self.__node.get_logger();
    

__all__: list[str] = ["HeartBeatProcessor"];