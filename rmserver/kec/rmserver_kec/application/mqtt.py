import os;
import json;
import socket;
import paho.mqtt.client as mqtt;
from rclpy.node import Node;
from rclpy.impl.rcutils_logger import RcutilsLogger;
from typing import Any;

class Client:

    def __init__(self, node: Node) -> None:
        self.client: mqtt.Client
        self.__node: Node = node;
        self.__log: RcutilsLogger = self.__node.get_logger();
        self.__current_mqtt_file: str = self.__node.get_parameter(name="current_mqtt_file").get_parameter_value().string_value;
        
        mqtt_config: Any = self.load_mqtt_config();
        
        if mqtt_config != {}:
            self.__host: str = mqtt_config["host"];
            self.__port: int = mqtt_config["port"];
            self.__client_name: str = mqtt_config["client_name"];
            self.__client_keep_alive: int = 60;
            self.__user_name: str = mqtt_config["user_name"];
            self.__password: str = mqtt_config["password"];
            self.__type: str = mqtt_config["type"];
            self.__path: str = mqtt_config["path"];
        else:
            self.__log.error("Loading MQTT Config Failed");
            return;
        
        self.is_connected: bool = False;
        
    def load_mqtt_config(self) -> Any:
        home_directory: str = os.path.expanduser("~");
        mqtt_path: str = f"{home_directory}/{self.__current_mqtt_file}";
        mqtt_config: Any = {};
        
        try:
            with open(mqtt_path, "r", encoding="utf-8") as f:
                mqtt_config = json.load(f);
                self.__log.info(f"MQTT Path: {mqtt_path}");
            return mqtt_config;
        except FileNotFoundError as fne:
            self.__log.error(f"{fne}");
            return {};

    def check_broker_opened(self) -> bool:
        try:
            sock: socket = socket.create_connection(address=(self.__host, self.__port), timeout=None);
            sock.close();
            return True;
        except socket.error as se:
            self.__log.error(f"MQTT socket error : {se}");
            pass;

        return False;

    def connect(self) -> None:
        try:
            self.__log.info(
                f"MQTT Connect\n"
                f"host : {self.__host}\n"
                f"port : {self.__port}\n"
                f"type : {self.__type}\n");

            self.client = mqtt.Client(callback_api_version=mqtt.CallbackAPIVersion.VERSION2, client_id=self.__client_name, clean_session=True, userdata=None, transport=self.__type);
            self.client.ws_set_options(path="/mqtt");
                
            self.client.username_pw_set(self.__user_name, self.__password);

            self.client.on_connect = self.on_connect;
            self.client.on_message = self.on_message;
            self.client.on_disconnect = self.on_disconnect;
            self.client.connect(host=self.__host, port=self.__port, keepalive=self.__client_keep_alive);
        except OSError as ose:
            self.__log.error(f"MQTT OSError : {ose}");
            return;
        except Exception as e:
            self.__log.error(f"MQTT Error : {e}");
            return;

    def run(self) -> None:
        self.client.loop_start();

    def on_connect(self, client, userdata, flags, reason_code, properties):
        if reason_code.is_failure:
            self.__log.error(f"Failed to connect: {reason_code}. loop_forever() will retry connection");
        else:
            self.is_connected = True;
            self.__log.info(f"MQTT Succeeded to Connect");

    def on_disconnect(self, client, userdata, flags, reason_code, properties) -> None:
        if reason_code:
            self.__log.error(f"MQTT disconnection result code : [{str(reason_code)}] ");
            self.connect();
            self.run();
        else:
            return;

    def on_message(self, client: Any, user_data: Any, msg: Any) -> None:
        pass;

    def on_subscribe(self, client, userdata, mid, reason_code_list, properties) -> None:
        if reason_code_list[0].is_failure:
            self.__log.error(f"MQTT Broker rejected you subscription: {reason_code_list[0]}");
        else:
            self.__log.info(f"MQTT Broker granted the following QoS: {reason_code_list[0].value}");

    def on_unsubscribe(self, client, userdata, mid, reason_code_list, properties) -> None:
        if len(reason_code_list) == 0 or not reason_code_list[0].is_failure:
            self.__log.error("MQTT unsubscribe succeeded (if SUBACK is received in MQTTv3 it success)");
        else:
            self.__log.info(f"MQTT Broker replied with failure: {reason_code_list[0]}");

    def publish(self, topic: str, payload: Any, qos: int) -> None:
        self.client.publish(topic=topic, payload=payload, qos=qos);

    def subscribe(self, topic: str, qos: int) -> None:
        self.__log.info(f"MQTT granted subscription\n\ttopic : {topic}\n\tqos : {qos}");
        self.client.subscribe(topic=topic, qos=qos);


__all__ = ["Client"];