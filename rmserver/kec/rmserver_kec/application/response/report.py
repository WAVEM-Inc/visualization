import json;
from rclpy.node import Node;
from rclpy.impl.rcutils_logger import RcutilsLogger;
from rclpy.subscription import Subscription;
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup;
from rclpy.qos import qos_profile_system_default;
from ktp_data_msgs.msg import ErrorReport;
from ktp_data_msgs.msg import ControlReport;
from rmserver_kec.application.mqtt import Client;
from rmserver_kec.application.message.conversion import ros_message_to_json;


MQTT_DEFAULT_QOS: int = 0;
MQTT_ERROR_REPORT_RESPONSE_TOPIC: str = "/rms/ktp/dummy/response/error_report";
MQTT_CONTROL_REPORT_RESPONSE_TOPIC: str = "/rms/ktp/dummy/response/control_report";

ERROR_REPORT_TOPIC_NAME: str = "/rms/ktp/data/error_report";
CONTROL_REPORT_TOPIC_NAME: str = "/rms/ktp/data/control_report";


class ReportProcessor:
    
    def __init__(self, node: Node, mqtt_client: Client) -> None:
        self.__node: Node = node;
        self.__log: RcutilsLogger = self.__node.get_logger();
        self.__mqtt_client: Client = mqtt_client;
    
        error_report_subscription_cb_group: MutuallyExclusiveCallbackGroup = MutuallyExclusiveCallbackGroup();
        self.__error_report_subscription: Subscription = self.__node.create_subscription(
            topic=ERROR_REPORT_TOPIC_NAME,
            msg_type=ErrorReport,
            qos_profile=qos_profile_system_default,
            callback_group=error_report_subscription_cb_group,
            callback=self.error_report_subscription_cb
        );
    
        control_report_subscription_cb_group: MutuallyExclusiveCallbackGroup = MutuallyExclusiveCallbackGroup();
        self.__control_report_subscription: Subscription = self.__node.create_subscription(
            topic=CONTROL_REPORT_TOPIC_NAME,
            msg_type=ControlReport,
            qos_profile=qos_profile_system_default,
            callback_group=control_report_subscription_cb_group,
            callback=self.control_report_subscription_cb
        );

    def error_report_subscription_cb(self, error_report_cb: ErrorReport) -> None:
        payload: str = ros_message_to_json(log=self.__log, ros_message=error_report_cb);
        self.__mqtt_client.publish(topic=MQTT_ERROR_REPORT_RESPONSE_TOPIC, payload=payload, qos=0);

    def control_report_subscription_cb(self, control_report_cb: ControlReport) -> None:
        payload: str = ros_message_to_json(log=self.__log, ros_message=control_report_cb);
        self.__mqtt_client.publish(topic=MQTT_CONTROL_REPORT_RESPONSE_TOPIC, payload=payload, qos=0);
        
        
__all__: list[str] = ["ReportProcessor"];