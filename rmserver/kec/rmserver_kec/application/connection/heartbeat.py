import json;
import paho.mqtt.client as paho;
from rclpy.node import Node;
from rclpy.impl.rcutils_logger import RcutilsLogger;
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup;
from rclpy.timer import Timer;
from rmserver_kec.application.mqtt import Client;

class HeartBeatProcessor:
    
    def __init__(self, node: Node, mqtt_client: Client) -> None:
        self.__node: Node = node;
        self.__log: RcutilsLogger = self.__node.get_logger();
        
        heartbeat_timer_cb_group: MutuallyExclusiveCallbackGroup = MutuallyExclusiveCallbackGroup();
        self.__heartbeat_timer: Timer = self.__node.create_timer(
            timer_period_sec=0.5,
            callback=self.heartbeat_timer_cb,
            callback_group=heartbeat_timer_cb_group
        );
    
    def heartbeat_timer_cb(self) -> None:
        pass;


__all__: list[str] = ["HeartBeatProcessor"];