from rclpy.node import Node;
from rclpy.impl.rcutils_logger import RcutilsLogger;
from rclpy.subscription import Subscription;
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup;
from rclpy.qos import qos_profile_system_default;
from ktp_dummy_interface.application.mqtt import Client;
from ktp_dummy_interface.application.message.conversion import ros_message_to_json;
from ktp_data_msgs.msg import GraphList;


MQTT_DEFAULT_QOS: int = 0;
MQTT_GRAPH_LIST_RESPONSE_TOPIC: str = "/rms/ktp/dummy/response/graph_list";

GRAPH_LIST_TOPIC_NAME: str = "/rms/ktp/data/graph_list";


class GraphProcessor:
    
    def __init__(self, node: Node, mqtt_client: Client) -> None:
        self.__node: Node = node;
        self.__log: RcutilsLogger = self.__node.get_logger();
        self.__mqtt_client: Client = mqtt_client;
        
        graph_list_subscription_cb_group: MutuallyExclusiveCallbackGroup = MutuallyExclusiveCallbackGroup();
        self.__graph_list_subscription: Subscription = self.__node.create_subscription(
            topic=GRAPH_LIST_TOPIC_NAME,
            msg_type=GraphList,
            qos_profile=qos_profile_system_default,
            callback_group=graph_list_subscription_cb_group,
            callback=self.graph_list_subscription_cb
        );
    
    def graph_list_subscription_cb(self, graph_list_cb: GraphList) -> None:
        payload: str = ros_message_to_json(log=self.__log, ros_message=graph_list_cb);
        self.__mqtt_client.publish(topic=MQTT_GRAPH_LIST_RESPONSE_TOPIC, payload=payload, qos=0);
        
__all__: list[str] = ["GraphProcessor"];