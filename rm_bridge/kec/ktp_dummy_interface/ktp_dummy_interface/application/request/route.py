import json;
import paho.mqtt.client as paho;
from rclpy.node import Node;
from rclpy.impl.rcutils_logger import RcutilsLogger;
from rclpy.publisher import Publisher;
import rclpy.action as rclpy_action;
from rclpy.action.client import ClientGoalHandle;
from rclpy.action.client import ActionClient;
from rclpy.task import Future;
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup;
from rclpy.qos import qos_profile_system_default;
from rosbridge_library.internal import message_conversion;
from std_msgs.msg import String;
from can_msgs.msg import Emergency;
from route_msgs.action import RouteToPose;
from action_msgs.msg import GoalStatus;
from typing import Dict;
from typing import Any;
from ktp_dummy_interface.application.message.conversion import json_to_ros_message;
from ktp_dummy_interface.application.mqtt import Client;
from ktp_dummy_interface.domain.route import RouteStatus;

MQTT_PATH_TOPIC: str = "/rms/ktp/dummy/response/path";
MQTT_ROUTE_STATUS_TOPIC: str = "/rms/ktp/dummy/response/route/status";

CAN_EMERGENCY_STOP_TOPIC: str = "/drive/can/emergency";
ROUTE_TO_POSE_ACTION: str = "/route_to_pose";


class RouteProcessor:
    
    def __init__(self, node: Node, mqtt_client: Client) -> None:
        self.__node: Node = node;
        self.__log: RcutilsLogger = self.__node.get_logger();
        self.__mqtt_client: Client = mqtt_client;
        
        self.__route_status: RouteStatus = RouteStatus();
        
        can_emergency_stop_cb_group: MutuallyExclusiveCallbackGroup = MutuallyExclusiveCallbackGroup();
        self.__can_emergency_stop_publisher: Publisher = self.__node.create_publisher(
            topic=CAN_EMERGENCY_STOP_TOPIC,
            msg_type=Emergency,
            qos_profile=qos_profile_system_default,
            callback_group=can_emergency_stop_cb_group
        );
        
        self.__route_to_pose_goal_index: int = 0;
        self.__route_to_pose_goal_list: list[RouteToPose.Goal] = [];
        self.__route_to_pose_goal_handle: ClientGoalHandle = None;
        
        route_to_pose_action_client_cb_group: MutuallyExclusiveCallbackGroup = MutuallyExclusiveCallbackGroup();
        self.__route_to_pose_action_client: ActionClient = rclpy_action.ActionClient(
            node=self.__node,
            action_name=ROUTE_TO_POSE_ACTION,
            action_type=RouteToPose,
            callback_group=route_to_pose_action_client_cb_group
        );
    
    def can_emergency_publish(self, emergency: Emergency) -> None:
        if emergency.stop is True:
            self.__log.info(f"{CAN_EMERGENCY_STOP_TOPIC} emergency stop...");
            self.__can_emergency_stop_publisher.publish(msg=emergency);
        else:
            return; 
        
    def mqtt_can_emergency_cb(self, mqtt_client: paho.Client, mqtt_user_data: Dict, mqtt_message: paho.MQTTMessage) -> None:
        try:
            mqtt_topic: str = mqtt_message.topic;
            mqtt_decoded_payload: str = mqtt_message.payload.decode();
            mqtt_json: Any = json.loads(mqtt_message.payload);

            emergency: String = message_conversion.populate_instance(msg=mqtt_json, inst=Emergency());

            self.__log.info(f"{mqtt_topic} cb\n{json.dumps(obj=message_conversion.extract_values(inst=emergency), indent=4)}");

            self.can_emergency_publish(emergency=emergency);
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
        
    def mqtt_route_to_pose_cb(self, mqtt_client: paho.Client, mqtt_user_data: Dict, mqtt_message: paho.MQTTMessage) -> None:
        try:
            mqtt_topic: str = mqtt_message.topic;
            mqtt_decoded_payload: str = mqtt_message.payload.decode();
            mqtt_json: Any = json.loads(mqtt_message.payload);
            node_list: list[Any] = mqtt_json["node_list"];
            
            for node in node_list:
                # goal: RouteToPose.Goal = message_conversion.populate_instance(msg=json.loads(json.dumps(node)), inst=RouteToPose.Goal());
                goal: RouteToPose.Goal = json_to_ros_message(log=self.__log, json_payload=json.loads(json.dumps(node)), target_ros_class=RouteToPose.Goal);
                self.__log.info(f"{mqtt_topic} cb\n{json.dumps(obj=message_conversion.extract_values(inst=goal), indent=4)}");
                self.__route_to_pose_goal_list.append(goal);

            self.__mqtt_client.publish(topic=MQTT_PATH_TOPIC, payload=json.dumps(mqtt_json), qos=0);
            self.route_to_pose_send_goal();
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
        
    def route_to_pose_send_goal(self) -> None:
        goal: RouteToPose.Goal = self.__route_to_pose_goal_list[self.__route_to_pose_goal_index];
        
        self.__log.info(f"{ROUTE_TO_POSE_ACTION} Send Goal[{self.__route_to_pose_goal_index}]\n{json.dumps(obj=message_conversion.extract_values(inst=goal), indent=4)}");

        if self.__route_to_pose_action_client.wait_for_server(timeout_sec=0.75):
            self.__route_to_pose_send_goal_future = self.__route_to_pose_action_client.send_goal_async(goal=goal, feedback_callback=self.route_to_pose_feedback_cb);
            self.__route_to_pose_send_goal_future.add_done_callback(callback=self.goal_response_callback);
        else:
            self.__log.error(f"{ROUTE_TO_POSE_ACTION} is not ready...");
            
    def route_to_pose_notify_status(self, driving_flag: bool, status_code: int) -> None:
        self.__route_status.is_driving = driving_flag;
        self.__route_status.node_index = self.__route_to_pose_goal_index;
        self.__route_status.status_code = status_code;
        current_goal: RouteToPose.Goal = self.__route_to_pose_goal_list[self.__route_to_pose_goal_index];
        self.__route_status.node_info = [current_goal.start_node.node_id, current_goal.end_node.node_id];
            
        payload: str = json.dumps(obj=self.__route_status.__dict__, indent=4);
        self.__log.info(f"{MQTT_ROUTE_STATUS_TOPIC} payload : {payload}");
        self.__mqtt_client.publish(topic=MQTT_ROUTE_STATUS_TOPIC, payload=payload, qos=0);
        

    def goal_response_callback(self, future: Future) -> None:
        self.__route_to_pose_goal_handle: ClientGoalHandle = future.result();
        if not self.__route_to_pose_goal_handle.accepted:
            self.__log.info(f"{ROUTE_TO_POSE_ACTION} Goal rejected");
            return;

        self.__log.info(f"{ROUTE_TO_POSE_ACTION} Goal accepted");

        self.__route_to_pose_action_get_result_future = self.__route_to_pose_goal_handle.get_result_async();
        self.__route_to_pose_action_get_result_future.add_done_callback(callback=self.route_to_pose_get_result_cb);

    def route_to_pose_feedback_cb(self, feedback_msg: RouteToPose.Impl.FeedbackMessage) -> None:
        feedback: RouteToPose.Feedback = feedback_msg.feedback;
        self.__log.info(f"{ROUTE_TO_POSE_ACTION} feedback cb\n{json.dumps(obj=message_conversion.extract_values(inst=feedback), indent=4)}");
        
        status_code: int = feedback.status_code;
        
        if status_code == 1001:
            self.route_to_pose_notify_status(driving_flag=True, status_code=0);
        else:
            return;
        
    def route_to_pose_get_result_cb(self, future: Future) -> None:
        result: RouteToPose.Result = future.result().result;
        status: int = future.result().status;

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.__log.info(f"{ROUTE_TO_POSE_ACTION} Goal succeeded!");
        else:
            self.__log.error(f"{ROUTE_TO_POSE_ACTION} Goal failed");
            
        if result.result == 1001:
            self.__log.info(f"{ROUTE_TO_POSE_ACTION} Goal[{self.__route_to_pose_goal_index}] Arrived");
            
            is_finished: bool = (self.__route_to_pose_goal_index == len(self.__route_to_pose_goal_list) - 1);
            if is_finished:
                self.route_to_pose_notify_status(driving_flag=False, status_code=2);
                self.__log.info(f"{ROUTE_TO_POSE_ACTION} navigation finished...");
                self.__route_to_pose_goal_index = 0;
                self.__route_to_pose_goal_list = [];
            else:
                self.route_to_pose_notify_status(driving_flag=False, status_code=1);
                self.__route_to_pose_goal_index = self.__route_to_pose_goal_index + 1;
                self.__log.info(f"{ROUTE_TO_POSE_ACTION} It will proceed Next Goal [{self.__route_to_pose_goal_index}]");
                self.route_to_pose_send_goal();
                
    def mqtt_goal_cancel_cb(self, mqtt_client: paho.Client, mqtt_user_data: Dict, mqtt_message: paho.MQTTMessage) -> None:
        try:
            mqtt_topic: str = mqtt_message.topic;
            mqtt_decoded_payload: str = mqtt_message.payload.decode();
            mqtt_json: Any = json.loads(mqtt_message.payload);
            
            self.__log.info(f"{ROUTE_TO_POSE_ACTION} cancel callback");
            self.__log.info(f"\n{json.dumps(mqtt_json, indent=4)}");
            
            if len(self.__route_to_pose_goal_list) != 0:
                self.route_to_pose_notify_status(driving_flag=False, status_code=5);
                self.__route_to_pose_goal_list = [];
                self.__route_to_pose_goal_index = 0;
                
                if self.__route_to_pose_goal_handle != None:
                    self.__log.info(f"{ROUTE_TO_POSE_ACTION} goal_handle : {json.dumps(message_conversion.extract_values(inst=self.__route_to_pose_goal_handle), 4)}");
                    cancel_goal_future: Future = self.__route_to_pose_action_client.cancel_goal_async(self.__route_to_pose_goal_handle);
                else:
                    self.__log.error(f"{ROUTE_TO_POSE_ACTION} goal_handle is None");
                    
                self.__route_to_pose_goal_handle = None;
                self.__log.info(f"{ROUTE_TO_POSE_ACTION} goal cancelled");
            else:
                self.__log.error(f"{ROUTE_TO_POSE_ACTION} goal is None");
                return;
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

__all__: list[str] = ["RouteProcessor"];