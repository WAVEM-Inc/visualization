import json;
import paho.mqtt.client as paho_mqtt;
from rclpy.node import Node;
from rclpy.impl.rcutils_logger import RcutilsLogger;
from rclpy.client import Client;
from rclpy.publisher import Publisher;
import rclpy.action as rclpy_action;
from rclpy.action.client import ClientGoalHandle;
from rclpy.action.client import ActionClient;
from rclpy.task import Future;
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup;
from rclpy.qos import qos_profile_system_default;
from rclpy.qos import qos_profile_services_default;
from rosbridge_library.internal import message_conversion;
from ktp_data_msgs.msg import Control;
from ktp_data_msgs.srv import AssignControl;
from ktp_data_msgs.msg import Mission;
from ktp_data_msgs.srv import AssignMission;
from ktp_data_msgs.msg import DetectedObject;
from std_msgs.msg import String;
from obstacle_msgs.msg import Status;
from can_msgs.msg import Emergency;
from route_msgs.action import RouteToPose;
import route_msgs.msg as route;
from action_msgs.msg import GoalStatus;
from rmserver_kec.application import mqtt;
from typing import Dict;
from typing import Any;
from rmserver_kec.application.message.conversion import json_to_ros_message;

ASSIGN_CONTROL_SERVICE_NAME: str = "/ktp_data_manager/assign/control";
ASSIGN_MISSION_SERVICE_NAME: str = "/ktp_data_manager/assign/mission";


class TaskProcessor:
    
    def __init__(self, node: Node) -> None:
        self.__node: Node = node;
        self.__log: RcutilsLogger = self.__node.get_logger();
        
        assign_control_service_client_cb_group: MutuallyExclusiveCallbackGroup = MutuallyExclusiveCallbackGroup();
        self.__assign_control_service_client: Client = self.__node.create_client(
            srv_name=ASSIGN_CONTROL_SERVICE_NAME,
            srv_type=AssignControl,
            qos_profile=qos_profile_services_default,
            callback_group=assign_control_service_client_cb_group
        );

        assign_mission_service_client_cb_group: MutuallyExclusiveCallbackGroup = MutuallyExclusiveCallbackGroup();
        self.__assign_mission_service_client: Client = self.__node.create_client(
            srv_name=ASSIGN_MISSION_SERVICE_NAME,
            srv_type=AssignMission,
            qos_profile=qos_profile_services_default,
            callback_group=assign_mission_service_client_cb_group
        );
    
    def mqtt_control_request_cb(self, mqtt_client: paho_mqtt.Client, mqtt_user_data: Dict, mqtt_message: paho_mqtt.MQTTMessage) -> None:
        try:
            mqtt_topic: str = mqtt_message.topic;
            mqtt_decoded_payload: str = mqtt_message.payload.decode();
            mqtt_json: Any = json.loads(mqtt_message.payload);

            control: Control = message_conversion.populate_instance(msg=mqtt_json, inst=Control());
            self.__log.info(f"{mqtt_topic} cb\n{json.dumps(obj=message_conversion.extract_values(inst=control), indent=4)}");

            self.assign_control_service_request(control=control);
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
    
    def assign_control_service_request(self, control: Control) -> None:
        assign_control_request: AssignControl.Request = AssignControl.Request();
        assign_control_request.control = control;

        is_assign_control_service_server_ready: bool = self.__assign_control_service_client.wait_for_service(timeout_sec=0.8);

        if is_assign_control_service_server_ready:
            assign_control_response: AssignControl.Response = self.__assign_control_service_client.call(request=assign_control_request);
            self.__log.info(f"{ASSIGN_CONTROL_SERVICE_NAME} Response : {json.dumps(message_conversion.extract_values(inst=assign_control_response), indent=4)}");

            if assign_control_response is None:
                self.__log.error(f"{ASSIGN_CONTROL_SERVICE_NAME} Response is None...");
                return;
        else:
            self.__log.error(f"{ASSIGN_CONTROL_SERVICE_NAME} Service Server is Not Ready...");
            return;

    def mqtt_mission_request_cb(self, mqtt_client: paho_mqtt.Client, mqtt_user_data: Dict, mqtt_message: paho_mqtt.MQTTMessage) -> None:
        try:
            mqtt_topic: str = mqtt_message.topic;
            mqtt_decoded_payload: str = mqtt_message.payload.decode();
            mqtt_json: Any = json.loads(mqtt_message.payload);

            mission: Mission = message_conversion.populate_instance(msg=mqtt_json, inst=Mission());
            self.__log.info(f"{mqtt_topic} cb\n{json.dumps(obj=message_conversion.extract_values(inst=mission), indent=4)}");

            self.assign_mission_service_request(mission=mission);

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

    def assign_mission_service_request(self, mission: Mission) -> None:
        assign_mission_request: AssignMission.Request = AssignMission.Request();
        assign_mission_request.mission = mission;

        is_assign_mission_service_server_ready: bool = self.__assign_mission_service_client.wait_for_service(timeout_sec=0.8);

        if is_assign_mission_service_server_ready:
            assign_mission_response: AssignControl.Response = self.__assign_mission_service_client.call(request=assign_mission_request);
            self.__log.info(f"{ASSIGN_MISSION_SERVICE_NAME} Response : {json.dumps(message_conversion.extract_values(inst=assign_mission_response), indent=4)}");

            if assign_mission_response is None:
                self.__log.error(f"{ASSIGN_MISSION_SERVICE_NAME} Response is None...");
                return;
        else:
            self.__log.error(f"{ASSIGN_MISSION_SERVICE_NAME} Service Server is Not Ready...");
            return;
    

__all__: list[str] = ["TaskProcessor"];