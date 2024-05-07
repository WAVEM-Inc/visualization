from rclpy.node import Node;
from rclpy.impl.rcutils_logger import RcutilsLogger;
from rmserver_kec.application.mqtt import Client;
from rmserver_kec.application.request.task import TaskProcessor;
from rmserver_kec.application.request.obstacle import ObstacleProcessor;
from rmserver_kec.application.request.route import RouteProcessor;
from rmserver_kec.application.request.path import PathProcessor;
from rmserver_kec.application.connection.heartbeat import HeartBeatProcessor;

MQTT_CONTROL_REQUEST_TOPIC: str = "/rms/ktp/dummy/request/control";
MQTT_MISSION_REQUEST_TOPIC: str = "/rms/ktp/dummy/request/mission";
MQTT_DETECTED_OBJECT_REQUEST_TOPIC: str = "/rms/ktp/dummy/request/detected_object";
MQTT_OBSTACLE_STATUS_TOPIC: str = "/rms/ktp/dummy/request/obstacle/status";
MQTT_DRIVE_OBSTACLE_COOPERATIVE_TOPIC: str = "/rms/ktp/dummy/request/obstacle/cooperative";
MQTT_CAN_EMERGENCY_STOP_TOPIC: str = "/rms/ktp/dummy/request/can/emergency";
MQTT_ROUTE_TO_POSE_TOPIC: str = "/rms/ktp/dummy/request/route_to_pose";
MQTT_ROUTE_TO_POSE_STATUS_TOPIC: str = "/rms/ktp/dummy/response/route_to_pose/status";
MQTT_GOAL_CANCEL_TOPIC: str = "/rms/ktp/dummy/request/goal/cancel";
MQTT_CAN_INIT_TOPIC: str = "/rms/ktp/dummy/request/can/init";
MQTT_PATH_EDIT_REQUEST_TOPIC: str = "/rms/ktp/dummy/request/path/edit";
MQTT_HEARTBEAT_REQUEST_TOPIC: str = "/rms/ktp/dummy/request/heartbeat";


class RequestBridge:

    def __init__(self, node: Node, mqtt_client: Client) -> None:
        self.__node: Node = node;
        self.__mqtt_client: Client = mqtt_client;
        
        self.__task_processor: TaskProcessor = TaskProcessor(node=self.__node);
        self.__obstacle_processor: ObstacleProcessor = ObstacleProcessor(node=self.__node);
        self.__route_processor: RouteProcessor = RouteProcessor(node=self.__node, mqtt_client=self.__mqtt_client);
        self.__path_processor: PathProcessor = PathProcessor(node=self.__node, mqtt_client=self.__mqtt_client);
        self.__heartbeat_processor: HeartBeatProcessor = HeartBeatProcessor(node=self.__node, mqtt_client=self.__mqtt_client);

        self.mqtt_subscription_init();

    def mqtt_subscription_init(self) -> None:
        self.__mqtt_client.subscribe(topic=MQTT_CONTROL_REQUEST_TOPIC, qos=0);
        self.__mqtt_client.client.message_callback_add(sub=MQTT_CONTROL_REQUEST_TOPIC, callback=self.__task_processor.mqtt_control_request_cb);

        self.__mqtt_client.subscribe(topic=MQTT_MISSION_REQUEST_TOPIC, qos=0);
        self.__mqtt_client.client.message_callback_add(sub=MQTT_MISSION_REQUEST_TOPIC, callback=self.__task_processor.mqtt_mission_request_cb);
        
        self.__mqtt_client.subscribe(topic=MQTT_DETECTED_OBJECT_REQUEST_TOPIC, qos=0);
        self.__mqtt_client.client.message_callback_add(sub=MQTT_DETECTED_OBJECT_REQUEST_TOPIC, callback=self.__obstacle_processor.mqtt_detected_object_cb);

        self.__mqtt_client.subscribe(topic=MQTT_OBSTACLE_STATUS_TOPIC, qos=0);
        self.__mqtt_client.client.message_callback_add(sub=MQTT_OBSTACLE_STATUS_TOPIC, callback=self.__obstacle_processor.mqtt_obstacle_event_cb);

        self.__mqtt_client.subscribe(topic=MQTT_DRIVE_OBSTACLE_COOPERATIVE_TOPIC, qos=0);
        self.__mqtt_client.client.message_callback_add(sub=MQTT_DRIVE_OBSTACLE_COOPERATIVE_TOPIC, callback=self.__obstacle_processor.mqtt_drive_obstacle_cooperative_cb);
        
        self.__mqtt_client.subscribe(topic=MQTT_CAN_EMERGENCY_STOP_TOPIC, qos=0);
        self.__mqtt_client.client.message_callback_add(sub=MQTT_CAN_EMERGENCY_STOP_TOPIC, callback=self.__route_processor.mqtt_can_emergency_cb);
        
        self.__mqtt_client.subscribe(topic=MQTT_ROUTE_TO_POSE_TOPIC, qos=0);
        self.__mqtt_client.client.message_callback_add(sub=MQTT_ROUTE_TO_POSE_TOPIC, callback=self.__route_processor.mqtt_route_to_pose_cb);
        
        self.__mqtt_client.subscribe(topic=MQTT_GOAL_CANCEL_TOPIC, qos=0);
        self.__mqtt_client.client.message_callback_add(sub=MQTT_GOAL_CANCEL_TOPIC, callback=self.__route_processor.mqtt_goal_cancel_cb);
        
        self.__mqtt_client.subscribe(topic=MQTT_CAN_INIT_TOPIC, qos=0);
        self.__mqtt_client.client.message_callback_add(sub=MQTT_CAN_INIT_TOPIC, callback=self.__route_processor.mqtt_can_init_cb);
        
        self.__mqtt_client.subscribe(topic=MQTT_PATH_EDIT_REQUEST_TOPIC, qos=0);
        self.__mqtt_client.client.message_callback_add(sub=MQTT_PATH_EDIT_REQUEST_TOPIC, callback=self.__path_processor.mqtt_path_edit_request_cb);
        
        self.__mqtt_client.subscribe(topic=MQTT_HEARTBEAT_REQUEST_TOPIC, qos=0);
        self.__mqtt_client.client.message_callback_add(sub=MQTT_HEARTBEAT_REQUEST_TOPIC, callback=self.__heartbeat_processor.mqtt_heart_beat_subscription_cb);
    

__all__: list[str] = ["RequestBridge"];