from rclpy.node import Node;
from rclpy.impl.rcutils_logger import RcutilsLogger;
from rclpy.subscription import Subscription;
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup;
from rclpy.qos import qos_profile_system_default;
from ktp_data_msgs.msg import Status;
from ktp_data_msgs.msg import ServiceStatus;
from ktp_dummy_interface.application.mqtt import Client;
from ktp_dummy_interface.application.message.conversion import ros_message_to_json;


MQTT_DEFAULT_QOS: int = 0;
MQTT_RBT_STATUS_RESPONSE_TOPIC: str = "/rms/ktp/dummy/response/rbt_status";
MQTT_SERVICE_STATUS_RESPONSE_TOPIC: str = "/rms/ktp/dummy/response/service_status";

RBT_STATUS_TOPIC_NAME: str = "/rms/ktp/data/rbt_status";
SERVICE_STATUS_TOPIC_NAME: str = "/rms/ktp/data/service_status";

class StatusProcessor:
    
    def __init__(self, node: Node, mqtt_client: Client) -> None:
        self.__node: Node = node;
        self.__log: RcutilsLogger = self.__node.get_logger();
        self.__mqtt_client: Client = mqtt_client;
        
        rbt_status_subscription_cb_group: MutuallyExclusiveCallbackGroup = MutuallyExclusiveCallbackGroup();
        self.__rbt_status_subscription: Subscription = self.__node.create_subscription(
            topic=RBT_STATUS_TOPIC_NAME,
            msg_type=Status,
            qos_profile=qos_profile_system_default,
            callback_group=rbt_status_subscription_cb_group,
            callback=self.rbt_status_subscription_cb
        );
    
        service_status_subscription_cb_group: MutuallyExclusiveCallbackGroup = MutuallyExclusiveCallbackGroup();
        self.__service_status_subscription_cb: Subscription = self.__node.create_subscription(
            topic=SERVICE_STATUS_TOPIC_NAME,
            msg_type=ServiceStatus,
            qos_profile=qos_profile_system_default,
            callback_group=service_status_subscription_cb_group,
            callback=self.service_status_subscription_cb
        );
        
    def rbt_status_subscription_cb(self, rbt_status_cb: Status) -> None:
        payload: str = ros_message_to_json(log=self.__log, ros_message=rbt_status_cb);
        self.__mqtt_client.publish(topic=MQTT_RBT_STATUS_RESPONSE_TOPIC, payload=payload, qos=0);

    def service_status_subscription_cb(self, service_status_cb: ServiceStatus) -> None:
        payload: str = ros_message_to_json(log=self.__log, ros_message=service_status_cb);
        self.__mqtt_client.publish(topic=MQTT_SERVICE_STATUS_RESPONSE_TOPIC, payload=payload, qos=0);
        
        
__all__: list[str] = ["StatusProcessor"];