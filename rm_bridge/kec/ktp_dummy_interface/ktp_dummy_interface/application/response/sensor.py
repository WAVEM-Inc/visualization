from rclpy.node import Node;
from rclpy.impl.rcutils_logger import RcutilsLogger;
from rclpy.subscription import Subscription;
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup;
from rclpy.qos import qos_profile_system_default;
from sensor_msgs.msg import NavSatFix;
from ktp_dummy_interface.application.mqtt import Client;
from ktp_dummy_interface.application.message.conversion import ros_message_to_json;


MQTT_DEFAULT_QOS: int = 0;
MQTT_GPS_RESPONSE_TOPIC: str = "/rms/ktp/dummy/response/gps";

UBLOX_FIX_TOPIC_NAME: str = "/sensor/ublox/fix";


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
        
    def ublox_fix_subscription_cb(self, ublox_fix_cb: NavSatFix) -> None:
        payload: str = ros_message_to_json(log=self.__log, ros_message=ublox_fix_cb);
        self.__mqtt_client.publish(topic=MQTT_GPS_RESPONSE_TOPIC, payload=payload, qos=0);
        
        
__all__: list[str] = ["SensorProcessor"];