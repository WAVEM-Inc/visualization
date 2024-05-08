import json;
import paho.mqtt.client as paho_mqtt;
from rclpy.node import Node;
from rclpy.impl.rcutils_logger import RcutilsLogger;
from rclpy.client import Client;
from rclpy.timer import Timer;
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup;
from rclpy.qos import qos_profile_services_default;
from rosbridge_library.internal import message_conversion;
from ktp_data_msgs.msg import Control;
from ktp_data_msgs.srv import AssignControl;
from ktp_data_msgs.msg import Mission;
from ktp_data_msgs.srv import AssignMission;
from typing import Dict;
from typing import Any;

from rmserver_kec.domain.task import set_control;
from rmserver_kec.domain.task import get_control_callback_flag;
from rmserver_kec.domain.task import set_control_callback_flag;
from rmserver_kec.domain.task import set_mission;
from rmserver_kec.domain.task import get_mission_callback_flag;
from rmserver_kec.domain.task import set_detected_object;
from rmserver_kec.domain.task import get_detected_object_flag;
from rmserver_kec.domain.task import set_mission_callback_flag;
from rmserver_kec.domain.task import get_control;
from rmserver_kec.domain.task import get_mission;
from rmserver_kec.domain.task import get_detected_object;
from rmserver_kec.domain.task import set_detected_object_flag;

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
        
        polling_timer_cb_group: MutuallyExclusiveCallbackGroup = MutuallyExclusiveCallbackGroup();
        self.__polling_timer: Timer = self.__node.create_timer(
            timer_period_sec=0.5,
            callback_group=polling_timer_cb_group,
            callback=self.__polling_timer_cb
        );
        
    def __polling_timer_cb(self) -> None:
        self.__node.get_logger().info(f"Waiting for Polling from KTP"
                                      f"\n\tcontrol : {get_control_callback_flag()}"
                                      f"\n\tmission : {get_mission_callback_flag()}"
                                      f"\n\tdetected_object : {get_detected_object_flag()}");
        
        if get_control_callback_flag():
            control: Control = message_conversion.populate_instance(msg=get_control(), inst=Control());
            self.__log.info(f"Control cb\n{json.dumps(obj=message_conversion.extract_values(inst=control), indent=4)}");
            self.assign_control_service_request(control=control);
            set_control_callback_flag(False);

        if get_mission_callback_flag():
            mission: Mission = message_conversion.populate_instance(msg=get_mission(), inst=Mission());
            self.__log.info(f"Mission cb\n{json.dumps(obj=message_conversion.extract_values(inst=mission), indent=4)}");
            self.assign_mission_service_request(mission=mission);
            set_mission_callback_flag(False);
        
    def mqtt_task_request_cb(self, mqtt_client: paho_mqtt.Client, mqtt_user_data: Dict, mqtt_message: paho_mqtt.MQTTMessage) -> None:
        try:
            mqtt_topic: str = mqtt_message.topic;
            mqtt_decoded_payload: str = mqtt_message.payload.decode();
            mqtt_json: Any = json.loads(mqtt_message.payload);
            
            resource_id: str = mqtt_json["id"];
            data: Any = json.loads(json.dumps(mqtt_json["data"]));
            self.__log.info(f"{mqtt_topic} cb\n{json.dumps(obj=data, indent=4)}");

            if resource_id == "rbt_control":
                print("!!! Control !!!");
                set_control_callback_flag(True);
                set_control(data);

                print(f"{resource_id} Callback : {json.dumps(get_control(), indent=4)}");
            elif resource_id == "rbt_mission":
                print("!!! Mission !!!");
                set_mission_callback_flag(True);
                set_mission(data);

                print(f"{resource_id} Callback : {json.dumps(get_mission(), indent=4)}");
            elif resource_id == "rbt_detected_object":
                print("!!! DetectedObject !!!");
                set_detected_object_flag(True);
                set_detected_object(data);

                print(f"{resource_id} Callback : {json.dumps(get_detected_object(), indent=4)}");
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