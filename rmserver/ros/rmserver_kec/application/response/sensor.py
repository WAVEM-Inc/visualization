from rclpy.node import Node;
from rclpy.impl.rcutils_logger import RcutilsLogger;
from rclpy.subscription import Subscription;
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup;
from rclpy.qos import qos_profile_system_default;
from rclpy.timer import Timer;
from sensor_msgs.msg import NavSatFix;
from sensor_msgs.msg import BatteryState;
from geometry_msgs.msg import PoseStamped;
from geometry_msgs.msg import Twist;
from rmserver_kec.application.mqtt import Client;
from rmserver_kec.application.message.conversion import ros_message_to_json;
from typing import Any;

MQTT_DEFAULT_QOS: int = 0;
MQTT_GPS_TOPIC: str = "/rmviz/response/gps";
MQTT_GPS_FILTERED_TOPIC: str = "/rmviz/response/gps/filtered";
MQTT_ODOM_EULAR_TOPIC: str = "/rmviz/response/odom/eular";
MQTT_BATTERY_STATE_TOPIC: str = "/rmviz/response/battery/state";
MQTT_CMD_VEL_TOPIC: str = "/rmviz/response/cmd_vel";

UBLOX_FIX_TOPIC_NAME: str = "/sensor/ublox/fix";
GPS_FILTERED_TOPIC_NAME: str = "/gps/filtered";
ODOM_EULAR_TOPIC_NAME: str = "/drive/odom/eular";
BATTERY_STATE_TOPIC_NAME: str = "/sensor/battery/state";
CMD_VEL_TOPIC_NAME: str = "/cmd_vel";


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
        
        gps_filtered_cb_group: MutuallyExclusiveCallbackGroup = MutuallyExclusiveCallbackGroup();
        self.__gps_filtered_subscription: Subscription = self.__node.create_subscription(
            topic=GPS_FILTERED_TOPIC_NAME,
            msg_type=NavSatFix,
            qos_profile=qos_profile_system_default,
            callback_group=gps_filtered_cb_group,
            callback=self.gps_filtered_subscription_cb
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
        
        battery_state_subscription_cb_group: MutuallyExclusiveCallbackGroup = MutuallyExclusiveCallbackGroup();
        self.__battery_state_subscription: Subscription = self.__node.create_subscription(
            topic=BATTERY_STATE_TOPIC_NAME,
            msg_type=BatteryState,
            qos_profile=qos_profile_system_default,
            callback_group=battery_state_subscription_cb_group,
            callback=self.battery_state_subscription_cb
        );
        
        cmd_vel_subscription_cb_group: MutuallyExclusiveCallbackGroup = MutuallyExclusiveCallbackGroup();
        self.__cmd_vel_subscription: Subscription = self.__node.create_subscription(
            topic=CMD_VEL_TOPIC_NAME,
            msg_type=Twist,
            qos_profile=qos_profile_system_default,
            callback_group=cmd_vel_subscription_cb_group,
            callback=self.cmd_vel_subscription_cb
        );
        
        self.__odom_eular_payload: str = None;
        
    def ublox_fix_subscription_cb(self, ublox_fix_cb: NavSatFix) -> None:
        # self.__log.info(f"{UBLOX_FIX_TOPIC_NAME} : {ublox_fix_cb}");
        payload: str = ros_message_to_json(log=self.__log, ros_message=ublox_fix_cb);
        self.__mqtt_client.publish(topic=MQTT_GPS_TOPIC, payload=payload, qos=0);
    
    def gps_filtered_subscription_cb(self, gps_filtered_cb: NavSatFix) -> None:
        payload: str = ros_message_to_json(log=self.__log, ros_message=gps_filtered_cb);
        self.__mqtt_client.publish(topic=MQTT_GPS_FILTERED_TOPIC, payload=payload, qos=0);
        
    def odom_eular_mqtt_publish_timer_cb(self) -> None:
        if self.__odom_eular_payload != None:
            self.__mqtt_client.publish(topic=MQTT_ODOM_EULAR_TOPIC, payload=self.__odom_eular_payload, qos=0);
        else:
            return;
    
    def odom_eular_subscription_cb(self, odom_eular_cb: PoseStamped) -> None:
        self.__odom_eular_payload = ros_message_to_json(log=self.__log, ros_message=odom_eular_cb);
        
    def battery_state_subscription_cb(self, battery_state_cb: BatteryState) -> None:
        payload: str = ros_message_to_json(log=self.__log, ros_message=battery_state_cb);
        self.__mqtt_client.publish(topic=MQTT_BATTERY_STATE_TOPIC, payload=payload, qos=0);
    
    def cmd_vel_subscription_cb(self, cmd_vel_cb: Twist) -> None:
        payload: str = ros_message_to_json(log=self.__log, ros_message=cmd_vel_cb);
        self.__mqtt_client.publish(topic=MQTT_CMD_VEL_TOPIC, payload=payload, qos=0);
        
__all__: list[str] = ["SensorProcessor"];