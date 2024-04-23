from rclpy.node import Node;
from rclpy.impl.rcutils_logger import RcutilsLogger;
from rclpy.subscription import Subscription;
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup;
from rclpy.qos import qos_profile_system_default;
from rclpy.timer import Timer;
from sensor_msgs.msg import NavSatFix;
from geometry_msgs.msg import PoseStamped;
from ktp_dummy_interface.application.mqtt import Client;
from ktp_dummy_interface.application.message.conversion import ros_message_to_json;
from typing import Any;

MQTT_DEFAULT_QOS: int = 0;
MQTT_GPS_RESPONSE_TOPIC: str = "/rms/ktp/dummy/response/gps";
MQTT_ODOM_EULAR_TOPIC: str = "/rms/ktp/dummy/response/odom/eular";

UBLOX_FIX_TOPIC_NAME: str = "/sensor/ublox/fix";
ODOM_EULAR_TOPIC_NAME: str = "/drive/odom/eular";


class SensorProcessor:
    
    def __init__(self, node: Node, mqtt_client: Client) -> None:
        self.__node: Node = node;
        self.__log: RcutilsLogger = self.__node.get_logger();
        self.__mqtt_client: Client = mqtt_client;
        
        ublox_fix_subscription_cb_group: MutuallyExclusiveCallbackGroup = MutuallyExclusiveCallbackGroup();
        self.__ublox_fix_subscription: Subscription = self.__node.create_subscription(
            topic=UBLOX_FIX_TOPIC_NAME,
            msg_type=NavSatFix,
            qos_profile=qos_profile_system_default,
            callback_group=ublox_fix_subscription_cb_group,
            callback=self.ublox_fix_subscription_cb
        );
        
        odom_eular_mqtt_publish_timer_cb_group: MutuallyExclusiveCallbackGroup = MutuallyExclusiveCallbackGroup();
        self.__odom_eular_mqtt_publish_timer: Timer = self.__node.create_timer(
            timer_period_sec=0.8,
            callback_group=odom_eular_mqtt_publish_timer_cb_group,
            callback=self.odom_eular_mqtt_publish_timer_cb
        );
        
        odom_eular_subscription_cb_group: MutuallyExclusiveCallbackGroup = MutuallyExclusiveCallbackGroup();
        self.__odom_eular_subscription: Subscription = self.__node.create_subscription(
            topic=ODOM_EULAR_TOPIC_NAME,
            msg_type=PoseStamped,
            qos_profile=qos_profile_system_default,
            callback_group=odom_eular_subscription_cb_group,
            callback=self.odom_eular_subscription_cb
        );
        
        self.__odom_eular_payload: str = None;
        
    def ublox_fix_subscription_cb(self, ublox_fix_cb: NavSatFix) -> None:
        payload: str = ros_message_to_json(log=self.__log, ros_message=ublox_fix_cb);
        self.__mqtt_client.publish(topic=MQTT_GPS_RESPONSE_TOPIC, payload=payload, qos=0);
    
    def odom_eular_mqtt_publish_timer_cb(self) -> None:
        if self.__odom_eular_payload != None:
            self.__mqtt_client.publish(topic=MQTT_ODOM_EULAR_TOPIC, payload=self.__odom_eular_payload, qos=0);
        else:
            return;
    
    def odom_eular_subscription_cb(self, odom_eular_cb: PoseStamped) -> None:
        self.__odom_eular_payload = ros_message_to_json(log=self.__log, ros_message=odom_eular_cb);
        
        
__all__: list[str] = ["SensorProcessor"];