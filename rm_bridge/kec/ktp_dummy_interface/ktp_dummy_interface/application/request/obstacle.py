import json;
import paho.mqtt.client as mqtt;
from rclpy.node import Node;
from rclpy.impl.rcutils_logger import RcutilsLogger;
from rclpy.publisher import Publisher;
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup;
from rclpy.qos import qos_profile_system_default;
from rosbridge_library.internal import message_conversion;
from ktp_data_msgs.msg import DetectedObject;
from std_msgs.msg import String;
from obstacle_msgs.msg import Status;
from typing import Dict;
from typing import Any;
from ktp_dummy_interface.application.message.conversion import json_to_ros_message;

DETECTED_OBJECT_TOPIC: str = "/rms/ktp/itf/detected_object";
OBSTACLE_EVENT_TOPIC: str = "/drive/obstacle/event";
DRIVE_OBSTACLE_COOPERATIVE_TOPIC: str = "/drive/obstacle/cooperative";


class ObstacleProcessor:
    
    def __init__(self, node: Node) -> None:
        self.__node: Node = node;
        self.__log: RcutilsLogger = self.__node.get_logger();
        
        detected_object_publisher_cb_group: MutuallyExclusiveCallbackGroup = MutuallyExclusiveCallbackGroup();
        self.__detected_object_publisher: Publisher = self.__node.create_publisher(
            topic=DETECTED_OBJECT_TOPIC,
            msg_type=DetectedObject,
            qos_profile=qos_profile_system_default,
            callback_group=detected_object_publisher_cb_group
        );
        
        obstacle_event_publisher_cb_group: MutuallyExclusiveCallbackGroup = MutuallyExclusiveCallbackGroup();
        self.__obstacle_event_publisher: Publisher = self.__node.create_publisher(
            topic=OBSTACLE_EVENT_TOPIC,
            msg_type=Status,
            qos_profile=qos_profile_system_default,
            callback_group=obstacle_event_publisher_cb_group
        );

        drive_obstacle_cooperative_publisher_cb_group: MutuallyExclusiveCallbackGroup = MutuallyExclusiveCallbackGroup();
        self.__drive_obstacle_cooperative_publisher: Publisher = self.__node.create_publisher(
            topic=DRIVE_OBSTACLE_COOPERATIVE_TOPIC,
            msg_type=String,
            qos_profile=qos_profile_system_default,
            callback_group=drive_obstacle_cooperative_publisher_cb_group
        );
        
    def mqtt_detected_object_cb(self, mqtt_client: mqtt.Client, mqtt_user_data: Dict, mqtt_message: mqtt.MQTTMessage) -> None:
        try:
            mqtt_topic: str = mqtt_message.topic;
            mqtt_decoded_payload: str = mqtt_message.payload.decode();
            mqtt_json: Any = json.loads(mqtt_message.payload);

            detected_object: DetectedObject = message_conversion.populate_instance(msg=mqtt_json, inst=DetectedObject());
            self.__log.info(f"{mqtt_topic} cb\n{json.dumps(obj=message_conversion.extract_values(inst=detected_object), indent=4)}");

            self.detected_object_publish(detected_object=detected_object);

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

    def detected_object_publish(self, detected_object: DetectedObject) -> None:
        self.__detected_object_publisher.publish(msg=detected_object);
        
    def mqtt_obstacle_event_cb(self, mqtt_client: mqtt.Client, mqtt_user_data: Dict, mqtt_message: mqtt.MQTTMessage) -> None:
        try:
            mqtt_topic: str = mqtt_message.topic;
            mqtt_decoded_payload: str = mqtt_message.payload.decode();
            mqtt_json: Any = json.loads(mqtt_message.payload);

            obstacle_status: Status = message_conversion.populate_instance(msg=mqtt_json, inst=Status());

            self.__log.info(f"{mqtt_topic} cb\n{json.dumps(obj=message_conversion.extract_values(inst=obstacle_status), indent=4)}");

            self.obstacle_event_publish(obstacle_status=obstacle_status);
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

    def obstacle_event_publish(self, obstacle_status: Status) -> None:
        self.__obstacle_event_publisher.publish(msg=obstacle_status);
        
    def mqtt_drive_obstacle_cooperative_cb(self, mqtt_client: mqtt.Client, mqtt_user_data: Dict, mqtt_message: mqtt.MQTTMessage) -> None:
        try:
            mqtt_topic: str = mqtt_message.topic;
            mqtt_decoded_payload: str = mqtt_message.payload.decode();
            mqtt_json: Any = json.loads(mqtt_message.payload);

            obstacle_cooperative: String = message_conversion.populate_instance(msg=mqtt_json, inst=String());

            self.__log.info(f"{mqtt_topic} cb\n{json.dumps(obj=message_conversion.extract_values(inst=obstacle_cooperative), indent=4)}");

            self.drive_obstacle_cooperative_publish(obstacle_cooperative=obstacle_cooperative);
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

    def drive_obstacle_cooperative_publish(self, obstacle_cooperative: String) -> None:
        self.__drive_obstacle_cooperative_publisher.publish(msg=obstacle_cooperative);
        

__all__: list[str] = ["ObstacleProcessor"];