import { MessagesMap, TypeClass } from "rclnodejs";

export const UBLOX_FIX_TOPIC: string = "/sensor/ublox/fix";
export const UBLOX_FIX_MSG_TYPE: TypeClass<keyof MessagesMap> = "sensor_msgs/msg/NavSatFix";
export const ODOM_EULAR_TOPIC: string = "/drive/odom/eular";
export const ODOM_EULAR_MSG_TYPE: TypeClass<keyof MessagesMap> = "geometry_msgs/msg/PoseStamped";
export const BATTERY_STATE_TOPIC: string = "/sensor/battery/state";
export const BATTERY_STATE_MSG_TYPE: TypeClass<keyof MessagesMap> = "sensor_msgs/msg/BatteryState";