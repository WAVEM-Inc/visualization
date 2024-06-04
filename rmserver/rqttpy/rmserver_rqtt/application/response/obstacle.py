import json;
import time;
from rclpy.node import Node;
from rclpy.impl.rcutils_logger import RcutilsLogger;
from rclpy.publisher import Publisher;
from rclpy.subscription import Subscription;
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup;
from rclpy.qos import qos_profile_system_default;
from rclpy.timer import Timer;
from rosbridge_library.internal import message_conversion;
from ktp_data_msgs.msg import ObstacleDetect;
from ktp_data_msgs.msg import LiDARSignal;
from ktp_data_msgs.msg import DetectedObject;
from rmserver_rqtt.application.mqtt import Client;
from typing import Any;
from ament_index_python.packages import get_package_share_directory;
from rmserver_rqtt.application.message.conversion import ros_message_to_json;


MQTT_DEFAULT_QOS: int = 0;
MQTT_OBSTACLE_DETECT_RESPONSE_TOPIC: str = "/rmviz/response/obstacle_detect";
MQTT_LIDAR_SIGNAL_RESPONSE_TOPIC: str = "/rmviz/response/lidar_signal";

OBSTACLE_DETECT_TOPIC_NAME: str = "/rms/ktp/data/obstacle_detect";
LIDAR_SIGNAL_TOPIC_NAME: str = "/rms/ktp/data/lidar_signal";
DETECTED_OBJECT_TOPIC_NAME: str = "/rms/ktp/itf/detected_object";
DRIVE_OBSTACLE_COOPERATIVE_TOPIC: str = "/drive/obstacle/cooperative";

class ObstacleProcessor:
    
    def __init__(self, node: Node, mqtt_client: Client) -> None:
        self.__node: Node = node;
        self.__log: RcutilsLogger = self.__node.get_logger();
        self.__mqtt_client: Client = mqtt_client;
        self.__lidar_signal_flag: bool = False;
        self.count: int = 0;        
    
        obstacle_detect_subscription_cb_group: MutuallyExclusiveCallbackGroup = MutuallyExclusiveCallbackGroup();
        self.__obstacle_detect_subscription: Subscription = self.__node.create_subscription(
            topic=OBSTACLE_DETECT_TOPIC_NAME,
            msg_type=ObstacleDetect,
            qos_profile=qos_profile_system_default,
            callback_group=obstacle_detect_subscription_cb_group,
            callback=self.obstacle_detect_subscription_cb
        );
    
        lidar_signal_subscription_cb_group: MutuallyExclusiveCallbackGroup = MutuallyExclusiveCallbackGroup();
        self.__lidar_signal_subscription: Subscription = self.__node.create_subscription(
            topic=LIDAR_SIGNAL_TOPIC_NAME,
            msg_type=LiDARSignal,
            qos_profile=qos_profile_system_default,
            callback_group=lidar_signal_subscription_cb_group,
            callback=self.lidar_signal_subscription_cb
        );
        
        detected_object_publisher_cb_group: MutuallyExclusiveCallbackGroup = MutuallyExclusiveCallbackGroup();
        self.__detected_object_publisher: Publisher = self.__node.create_publisher(
            msg_type=DetectedObject,
            topic=DETECTED_OBJECT_TOPIC_NAME,
            callback_group=detected_object_publisher_cb_group,
            qos_profile=qos_profile_system_default
        );
        
        detected_object_publish_timer_cb_group: MutuallyExclusiveCallbackGroup = MutuallyExclusiveCallbackGroup();
        self.__detected_object_publish_timer: Timer = self.__node.create_timer(
            timer_period_sec=0.5,
            callback_group=detected_object_publish_timer_cb_group,
            callback=self.detected_timer_cb
        );
        
        self.__detected_object: DetectedObject = self.load_and_convert_detected_object_json();
    
    def load_and_convert_detected_object_json(self) -> DetectedObject:
        try:
            package_shared_directory: str = get_package_share_directory(self.__node.get_name());
            self.__log.info(f"package_shared_directory : {package_shared_directory}");
            json_path: str = f"{package_shared_directory}/json/detected_object.json";
            self.__log.info(f"DETECTECD JSON PATH : {json_path}");
            
            json_data: Any = {};
            
            with open(json_path, "r") as f:
                json_data = json.load(f);
            
            # self.__log.info(f"DETECTECD JSON Data : {json.dumps(json_data, indent=4)}");
            detected_object: DetectedObject = message_conversion.populate_instance(msg=json_data, inst=DetectedObject());
            # self.__log.info(f"\n{json.dumps(obj=message_conversion.extract_values(inst=detected_object), indent=4)}");
            return detected_object;
        except ValueError as ve:
            self.__log.error(f"load_and_convert_detected_object_json : {ve}");
            return None;
        except KeyError as ke:
            self.__log.error(f"load_and_convert_detected_object_json : {ke}");
            return None;
        except message_conversion.NonexistentFieldException as nefe:
            self.__log.error(f"load_and_convert_detected_object_json : {nefe}");
            return None;
        except Exception as e:
            self.__log.error(f"load_and_convert_detected_object_json : {e}");
            return None;

    def obstacle_detect_subscription_cb(self, obstacle_detect_cb: ObstacleDetect) -> None:
        payload: str = ros_message_to_json(log=self.__log, ros_message=obstacle_detect_cb);
        self.__mqtt_client.publish(topic=MQTT_OBSTACLE_DETECT_RESPONSE_TOPIC, payload=payload, qos=0);
    
    def lidar_signal_subscription_cb(self, lidar_signal_cb: LiDARSignal) -> None:
        if lidar_signal_cb.signal_type == "START":
            self.__lidar_signal_flag = True;
        elif lidar_signal_cb.signal_type == "STOP":
            self.count = 0;
            self.__lidar_signal_flag = False;
        else:
            self.__lidar_signal_flag = False;

        payload: str = ros_message_to_json(log=self.__log, ros_message=lidar_signal_cb);
        self.__mqtt_client.publish(topic=MQTT_LIDAR_SIGNAL_RESPONSE_TOPIC, payload=payload, qos=0);
    
    def detected_timer_cb(self) -> None:
        if self.__lidar_signal_flag is True:
            if self.__detected_object != None:
                if self.count == 20:
                    self.__detected_object_publisher.publish(msg=DetectedObject());
                    self.__lidar_signal_flag = False;
                    self.count = 0;
                else:
                    self.__detected_object_publisher.publish(msg=self.__detected_object);
                self.count += 1;
                self.__log.info(f"{DETECTED_OBJECT_TOPIC_NAME} count : {self.count}");
            else:
                self.__log.error(f"{DETECTED_OBJECT_TOPIC_NAME} detected_object is None...");
                return;
        else:
            return;


__all__: list[str] = ["ObstacleProcessor"];