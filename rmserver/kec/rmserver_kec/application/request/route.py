import os;
import json;
import configparser;
import paho.mqtt.client as paho;
from rclpy.node import Node;
from rclpy.impl.rcutils_logger import RcutilsLogger;
from rclpy.publisher import Publisher;
from rclpy.subscription import Subscription;
import rclpy.action as rclpy_action;
from rclpy.action.client import ClientGoalHandle;
from rclpy.action.client import ActionClient;
from rclpy.task import Future;
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup;
from rclpy.qos import qos_profile_system_default;
from rosbridge_library.internal import message_conversion;
from std_msgs.msg import String;
from can_msgs.msg import Emergency;
from can_msgs.msg import Init;
from route_msgs.action import RouteToPose;
from route_msgs.msg import Position;
from route_msgs.msg import Node as route_Node;
from route_msgs.msg import DetectionRange;
from route_msgs.msg import Path;
from action_msgs.msg import GoalStatus;
from typing import Dict;
from typing import Any;
from rmserver_kec.application.message.conversion import json_to_ros_message;
from rmserver_kec.application.message.conversion import ros_message_to_json;
from rmserver_kec.application.mqtt import Client;
from rmserver_kec.domain.route import RouteStatus;

MQTT_PATH_TOPIC: str = "/rmviz/response/path";
MQTT_ROUTE_STATUS_TOPIC: str = "/rmviz/response/route/status";
MQTT_SELECT_PATH_FILE_RESPONSE_TOPIC: str = "/rmviz/response/path/select";

ROUTE_TO_POSE_ACTION: str = "/route_to_pose";
TASK_GOAL_CANCEL_TOPIC: str = "/rms/ktp/task/goal/cancel";
CAN_INIT_TOPIC: str = "/drive/can/init";
CAN_EMERGENCY_STOP_TOPIC: str = "/drive/can/emergency";
NOTIFY_PATH_TOPIC: str = "/rms/ktp/task/notify/path";


class RouteProcessor:
    
    def __init__(self, node: Node, mqtt_client: Client) -> None:
        self.__node: Node = node;
        self.__log: RcutilsLogger = self.__node.get_logger();
        self.__mqtt_client: Client = mqtt_client;
        self.__current_path_file: str = self.__node.get_parameter("current_map_config_file").get_parameter_value().string_value;
        self.__path: Any = {};
        
        self.__route_status: RouteStatus = RouteStatus();
        
        self.__route_to_pose_goal_index: int = 0;
        self.__route_to_pose_goal_list: list[RouteToPose.Goal] = [];
        self.__route_to_pose_goal_list_size: int = 0;
        self.__route_to_pose_goal_handle: ClientGoalHandle = None;
        
        route_to_pose_action_client_cb_group: MutuallyExclusiveCallbackGroup = MutuallyExclusiveCallbackGroup();
        self.__route_to_pose_action_client: ActionClient = rclpy_action.ActionClient(
            node=self.__node,
            action_name=ROUTE_TO_POSE_ACTION,
            action_type=RouteToPose,
            callback_group=route_to_pose_action_client_cb_group
        );
        
        task_goal_cancel_publisher_cb_group: MutuallyExclusiveCallbackGroup = MutuallyExclusiveCallbackGroup();
        self.__task_goal_cancel_publisher: Publisher = self.__node.create_publisher(
            topic=TASK_GOAL_CANCEL_TOPIC,
            msg_type=String,
            callback_group=task_goal_cancel_publisher_cb_group,
            qos_profile=qos_profile_system_default
        );
        
        can_init_publisher_cb_group: MutuallyExclusiveCallbackGroup = MutuallyExclusiveCallbackGroup();
        self.__can_init_publisher: Publisher = self.__node.create_publisher(
            topic=CAN_INIT_TOPIC,
            msg_type=Init,
            qos_profile=qos_profile_system_default,
            callback_group=can_init_publisher_cb_group
        );
        
        can_emergency_stop_publisher_cb_group: MutuallyExclusiveCallbackGroup = MutuallyExclusiveCallbackGroup();
        self.__can_emergency_stop_publisher: Publisher = self.__node.create_publisher(
            topic=CAN_EMERGENCY_STOP_TOPIC,
            msg_type=Emergency,
            qos_profile=qos_profile_system_default,
            callback_group=can_emergency_stop_publisher_cb_group
        );
        
        notify_path_subscription_cb_group: MutuallyExclusiveCallbackGroup = MutuallyExclusiveCallbackGroup();
        self.__notify_path_subscription: Subscription = self.__node.create_subscription(
            topic=NOTIFY_PATH_TOPIC,
            msg_type=Path,
            qos_profile=qos_profile_system_default,
            callback_group=notify_path_subscription_cb_group,
            callback=self.notify_path_subscription_cb
        );
        
        self.load_map();
    
    def load_map(self) -> None:
        home_directory: str = os.path.expanduser("~");
        map_config_path: str = f"{home_directory}/{self.__current_path_file}";
        self.__log.info(f"Map Config Path: {map_config_path}");
        
        config_parser: configparser.ConfigParser = configparser.ConfigParser();
        config_parser.read(filenames=map_config_path);
        
        map_file_path: str = config_parser["CONFIG"]["file_path"];
        map_file_name: str = config_parser["CONFIG"]["file_name"];
        
        map_file_full_path: str = f"{home_directory}{map_file_path}{map_file_name}";
        
        self.__log.info(f"Map Path : {map_file_full_path}");
        
        try:
            with open(map_file_full_path, "r", encoding="utf-8") as f:
                self.__path = json.load(f);
        except FileNotFoundError as fne:
            self.__log.error(f"{fne}");
            return;
        
    def mqtt_route_to_pose_cb(self, mqtt_client: paho.Client, mqtt_user_data: Dict, mqtt_message: paho.MQTTMessage) -> None:
        try:
            mqtt_topic: str = mqtt_message.topic;
            mqtt_decoded_payload: str = mqtt_message.payload.decode();
            mqtt_json: Any = json.loads(mqtt_message.payload);
            self.__log.info(f"{mqtt_topic}\n{json.dumps(mqtt_json, indent=4, ensure_ascii=False)}");
            
            path_array: list[Any] = self.__path["path"];
            
            self.__log.info(f"{ROUTE_TO_POSE_ACTION} route_to_pose_goal_list_size : {self.__route_to_pose_goal_list_size}");
            
            if self.__route_to_pose_goal_list_size == 0:
                path_id: str = mqtt_json["path"]["path_id"];
                path_name: str = mqtt_json["path"]["path_name"];
                self.__log.info(f"path_id : {path_id}, path_name : {path_name}");
                
                node_list: list[Any] = [];

                for path in path_array:
                    if path["id"] == path_id:
                        node_list = path["nodeList"];
                        break;
                    
                self.__route_to_pose_goal_list_size = len(node_list) - 1;
                    
                for i in range(len(node_list) - 1):
                    goal: RouteToPose.Goal = RouteToPose.Goal();
                    start_node: route_Node = route_Node();
                    end_node: route_Node = route_Node();
                    
                    if i == self.__route_to_pose_goal_list_size:
                        self.__log.info(f"{mqtt_topic} converting node to goal finished...");
                        self.__log.info(f"{mqtt_topic} goal list size : {self.__route_to_pose_goal_list_size}...");
                        break;
                    
                    _start_node: Any = node_list[i];
                    
                    start_node.node_id = _start_node["nodeId"].split("-")[2];
                    start_node.position = json_to_ros_message(log=self.__log, json_payload=json.loads(json.dumps(_start_node["position"])), target_ros_class=Position);
                    start_node.type = _start_node["type"];
                    start_node.kind = _start_node["kind"];
                    start_node.heading = float(_start_node["heading"]);
                    start_node.direction = _start_node["direction"];
                    start_node.driving_option = _start_node["drivingOption"];
                    
                    if len(_start_node["detectionRange"]) != 0:
                        for dr in _start_node["detectionRange"]:
                            detection_range: DetectionRange = DetectionRange();
                            detection_range.offset = dr["offset"];
                            detection_range.width_left = dr["widthLeft"];
                            detection_range.width_right = dr["widthRight"];
                            detection_range.height = dr["height"];
                            detection_range.action_code = dr["actionCode"];
                            start_node.detection_range.append(detection_range);
                    else:
                        start_node.detection_range = [];
                    
                    _end_node: Any = node_list[i+1];
                    
                    end_node.node_id = _end_node["nodeId"].split("-")[2];
                    end_node.position = json_to_ros_message(log=self.__log, json_payload=json.loads(json.dumps(_end_node["position"])), target_ros_class=Position);
                    end_node.type = _end_node["type"];
                    end_node.kind = _end_node["kind"];
                    end_node.heading = float(_end_node["heading"]);
                    end_node.direction = _end_node["direction"];
                    end_node.driving_option = _end_node["drivingOption"];
                    
                    if len(_end_node["detectionRange"]) != 0:
                        for dr in _end_node["detectionRange"]:
                            detection_range: DetectionRange = DetectionRange();
                            detection_range.offset = dr["offset"];
                            detection_range.width_left = dr["widthLeft"];
                            detection_range.width_right = dr["widthRight"];
                            detection_range.height = dr["height"];
                            detection_range.action_code = dr["actionCode"];
                            end_node.detection_range.append(detection_range);
                    else:
                        end_node.detection_range = [];
                
                    goal.start_node = start_node;
                    goal.end_node = end_node;
                    self.__route_to_pose_goal_list.append(goal);

                for [index, goal] in enumerate(self.__route_to_pose_goal_list):
                    self.__log.info(f"{mqtt_topic} goal[{index}]\n{json.dumps(message_conversion.extract_values(inst=goal), indent=4)}");
                    
                self.__mqtt_client.publish(topic=MQTT_PATH_TOPIC, payload=json.dumps(node_list), qos=0);
                
                if mqtt_json["isEnableToCommandRoute"] == "false":
                    self.__log.error(f"{ROUTE_TO_POSE_ACTION} cannot command route");
                    self.route_to_pose_flush_goal();
                    return;
                else:
                    self.route_to_pose_send_goal();
            else:
                self.__log.error(f"{mqtt_topic} navigation is already proceeding...");
                self.route_to_pose_notify_status(driving_flag=False, status_code=4);
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
        
    def route_to_pose_flush_goal(self) -> None:
        self.__route_to_pose_goal_index = 0;
        self.__route_to_pose_goal_list.clear();
        self.__route_to_pose_goal_list_size = 0;
        self.__log.info("============== Goal Flush ==============");
        
    def route_to_pose_send_goal(self) -> None:
        try:
            goal: RouteToPose.Goal = self.__route_to_pose_goal_list[self.__route_to_pose_goal_index];
        
            self.__log.info(f"{ROUTE_TO_POSE_ACTION} Send Goal[{self.__route_to_pose_goal_index} / {self.__route_to_pose_goal_list_size - 1}]\n{json.dumps(obj=message_conversion.extract_values(inst=goal), indent=4)}");

            if self.__route_to_pose_action_client.wait_for_server(timeout_sec=0.75):
                self.__route_to_pose_send_goal_future = self.__route_to_pose_action_client.send_goal_async(goal=goal, feedback_callback=self.route_to_pose_feedback_cb);
                self.__route_to_pose_send_goal_future.add_done_callback(callback=self.goal_response_callback);
            else:
                self.__log.error(f"{ROUTE_TO_POSE_ACTION} is not ready...");
                self.route_to_pose_notify_status(driving_flag=False, status_code=3);
                self.route_to_pose_flush_goal();
                return;
        except IndexError as ide:
            self.__log.error(f"{ROUTE_TO_POSE_ACTION} : {ide}");
            return;
            
    def route_to_pose_notify_status(self, driving_flag: bool, status_code: int) -> None:
        try:
            self.__route_status.is_driving = driving_flag;
            self.__route_status.status_code = status_code;
            
            if self.__route_to_pose_goal_list_size != 0:
                self.__route_status.node_index = self.__route_to_pose_goal_index;
                current_goal: RouteToPose.Goal = self.__route_to_pose_goal_list[self.__route_to_pose_goal_index];
                self.__route_status.node_info = [current_goal.start_node.node_id, current_goal.end_node.node_id];
                
            payload: str = json.dumps(obj=self.__route_status.__dict__, indent=4);
            self.__log.info(f"{MQTT_ROUTE_STATUS_TOPIC} payload : {payload}");
            self.__mqtt_client.publish(topic=MQTT_ROUTE_STATUS_TOPIC, payload=payload, qos=0);
        except IndexError as ide:
            self.__log.error(f"{MQTT_ROUTE_STATUS_TOPIC} : {ide}");
            return;

    def goal_response_callback(self, future: Future) -> None:
        route_to_pose_goal_handle = future.result();
        if not route_to_pose_goal_handle.accepted:
            self.__log.info(f"{ROUTE_TO_POSE_ACTION} Goal rejected");
            return;

        self.__log.info(f"{ROUTE_TO_POSE_ACTION} Goal accepted");
        
        self.__route_to_pose_goal_handle = route_to_pose_goal_handle;

        self.__route_to_pose_action_get_result_future = route_to_pose_goal_handle.get_result_async();
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
            self.__log.info(f"{ROUTE_TO_POSE_ACTION} Goal[{self.__route_to_pose_goal_index} / {self.__route_to_pose_goal_list_size - 1}] Arrived");
            
            is_finished: bool = (self.__route_to_pose_goal_index == self.__route_to_pose_goal_list_size - 1);
            if is_finished:
                self.route_to_pose_notify_status(driving_flag=False, status_code=2);
                self.__log.info(f"{ROUTE_TO_POSE_ACTION} navigation finished...");
                self.route_to_pose_flush_goal();
            else:
                self.route_to_pose_notify_status(driving_flag=False, status_code=1);
                self.__route_to_pose_goal_index = self.__route_to_pose_goal_index + 1;
                self.__log.info(f"{ROUTE_TO_POSE_ACTION} It will proceed Next Goal [{self.__route_to_pose_goal_index}]");
                self.route_to_pose_send_goal();
    
    def __route_to_pose_goal_cancel_cb(self, future) -> None:
        cancel_response = future.result();
        if len(cancel_response.goals_canceling) > 0:
            self.__log.info(f"{ROUTE_TO_POSE_ACTION} Goal successfully canceled");
        else:
            self.__log.error(f"{ROUTE_TO_POSE_ACTION} Goal failed canceling");
            
    def mqtt_goal_cancel_cb(self, mqtt_client: paho.Client, mqtt_user_data: Dict, mqtt_message: paho.MQTTMessage) -> None:
        try:
            mqtt_topic: str = mqtt_message.topic;
            mqtt_decoded_payload: str = mqtt_message.payload.decode();
            mqtt_json: Any = json.loads(mqtt_message.payload);
            
            self.__log.info(f"{ROUTE_TO_POSE_ACTION} cancel callback");
            self.__log.info(f"\n{json.dumps(mqtt_json, indent=4)}");
            
            self.route_to_pose_notify_status(driving_flag=False, status_code=5);
            self.route_to_pose_flush_goal();
            self.__task_goal_cancel_publisher.publish(msg=String());
            
            if self.__route_to_pose_goal_handle != None:
                self.__log.info(f"{ROUTE_TO_POSE_ACTION} goal_handle : {json.dumps(message_conversion.extract_values(inst=self.__route_to_pose_goal_handle), 4)}");
                cancel_goal_future: Future = self.__route_to_pose_goal_handle.cancel_goal_async();
                cancel_goal_future.add_done_callback(self.__route_to_pose_goal_cancel_cb);
            else:
                self.__log.error(f"{ROUTE_TO_POSE_ACTION} goal_handle is None");
            self.__route_to_pose_goal_handle = None;
            self.__log.info(f"{ROUTE_TO_POSE_ACTION} goal cancelled");
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
        
    def can_init_publish(self, init: Init) -> None:
        if init.can_sign_tran_state is True:
            self.__log.info(f"{CAN_EMERGENCY_STOP_TOPIC} Init...");
            self.__can_init_publisher.publish(msg=init);
        else:
            return;
        
    def mqtt_can_init_cb(self, mqtt_client: paho.Client, mqtt_user_data: Dict, mqtt_message: paho.MQTTMessage) -> None:
        try:
            mqtt_topic: str = mqtt_message.topic;
            mqtt_decoded_payload: str = mqtt_message.payload.decode();
            mqtt_json: Any = json.loads(mqtt_message.payload);
            
            init: Init = json_to_ros_message(log=self.__log, json_payload=mqtt_json, target_ros_class=Init);
            self.__log.info(f"{CAN_INIT_TOPIC} init : {init}");
            self.can_init_publish(init=init);
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
        
    def notify_path_subscription_cb(self, path_cb: Path) -> None:
        self.__log.info(f"{NOTIFY_PATH_TOPIC} cb\n{json.dumps(obj=message_conversion.extract_values(inst=path_cb), indent=4)}");
        
        node_list: list[route_Node] = path_cb.node_list;
        node_list_size: int = len(node_list);
        
        path_json_list: list[Any] = [];
        
        for [index, node] in enumerate(node_list):
            path_json: Any = {};
            self.__log.info(f"{NOTIFY_PATH_TOPIC} converting [{index} / {node_list_size}]");
            self.__log.info(f"{NOTIFY_PATH_TOPIC} cb\n{json.dumps(obj=message_conversion.extract_values(inst=node), indent=4)}");
            
            path_json["nodeId"] = node.node_id;
            path_json["position"] = json.loads(json.dumps(message_conversion.extract_values(inst=node.position)));
            path_json["type"] = node.type;
            path_json["kind"] = node.kind;
            path_json["heading"] = node.heading;
            path_json["direction"] = node.direction;
            path_json["drivingOption"] = node.driving_option;
            # path_json["detectionRange"] = node.detection_range;
            
            path_json_list.append(path_json);
            
            if index == node_list_size - 1:
                self.__log.info(f"{NOTIFY_PATH_TOPIC} path converting finished");
                break;
            
        for path in path_json_list:
            self.__log.info(f"{NOTIFY_PATH_TOPIC} path_json : {json.dumps(obj=path, indent=4)}");
            
        payload: str = json.dumps(obj=path_json_list);
        self.__mqtt_client.publish(topic=MQTT_PATH_TOPIC, payload=payload, qos=0);
    
    def mqtt_path_renew_cb(self, mqtt_client: paho.Client, mqtt_user_data: Dict, mqtt_message: paho.MQTTMessage) -> None:
        try:
            mqtt_topic: str = mqtt_message.topic;
            mqtt_decoded_payload: str = mqtt_message.payload.decode();
            mqtt_json: Any = json.loads(mqtt_message.payload);

            self.load_map();
            self.__log.info(f"{mqtt_topic} path renewed\n{json.dumps(obj=self.__path, indent=4)}");
            self.__log.info(f"=============================== Path Renewed ===============================");
        except KeyError as ke:
            self.__log.error(f"Invalid JSON Key in MQTT {mqtt_topic} subscription callback: {ke}");
            return;

        except json.JSONDecodeError as jde:
            self.__log.error(f"Invalid JSON format in MQTT {mqtt_topic} subscription callback: {jde.msg}");
            return;

        except message_conversion.NonexistentFieldException as nefe:
            self.__log.error(f"{mqtt_topic} : {nefe}");
            return;
        
        except OSError as ose:
            self.__log.error(f"{mqtt_topic} : {ose}");
            return;
        
        except Exception as e:
            self.__log.error(f"Exception in MQTT {mqtt_topic} subscription callback: {e}");
            return;
        
    def mqtt_path_select_cb(self, mqtt_client: paho.Client, mqtt_user_data: Dict, mqtt_message: paho.MQTTMessage) -> None:
        try:
            mqtt_topic: str = mqtt_message.topic;
            mqtt_decoded_payload: str = mqtt_message.payload.decode();
            mqtt_json: Any = json.loads(mqtt_message.payload);
            
            
            self.load_map();
            payload: str = json.dumps(obj=self.__path, indent=4);
            self.__log.info(f"=============================== Path Selected ===============================");
            self.__log.info(f"{mqtt_topic} path selected\n{payload}");
            self.__mqtt_client.publish(topic=MQTT_SELECT_PATH_FILE_RESPONSE_TOPIC, payload=payload, qos=0);
        except KeyError as ke:
            self.__log.error(f"Invalid JSON Key in MQTT {mqtt_topic} subscription callback: {ke}");
            return;

        except json.JSONDecodeError as jde:
            self.__log.error(f"Invalid JSON format in MQTT {mqtt_topic} subscription callback: {jde.msg}");
            return;

        except message_conversion.NonexistentFieldException as nefe:
            self.__log.error(f"{mqtt_topic} : {nefe}");
            return;
        
        except OSError as ose:
            self.__log.error(f"{mqtt_topic} : {ose}");
            return;
        
        except Exception as e:
            self.__log.error(f"Exception in MQTT {mqtt_topic} subscription callback: {e}");
            return;

__all__: list[str] = ["RouteProcessor"];