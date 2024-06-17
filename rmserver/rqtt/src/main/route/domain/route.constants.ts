import { ActionsMap, MessagesMap, TypeClass } from "rclnodejs";

export const ROUTE_TO_POSE_ACTION: string = "/route_to_pose";
export const ROUTE_TO_POSE_ACTION_TYPE: TypeClass<keyof ActionsMap> = "route_msgs/action/RouteToPose";
export const ROUTE_TO_POSE_ACTON_GOAL: TypeClass<keyof MessagesMap> = "route_msgs/action/RouteToPose_Goal";

export const TASK_GOAL_CANCEL_TOPIC: string = "/rms/ktp/task/goal/cancel";
export const TASK_GOAL_CANCEL_MSG_TYPE: TypeClass<keyof MessagesMap> = "std_msgs/msg/String";

export const ROUTE_PATH_TOPIC: string = "/route/path";
export const ROUTE_STATUS_TOPIC: string = "/route/status";

export const PATH_SELECT_TOPIC: string = "/path/select";
export const PATH_RENEW_TOPIC: string = "/path/renew";

export const GOAL_CANCEL_TOPIC: string = "/goal/cancel";

export const CAN_EMERGENCY_STOP_TOPIC: string = "/drive/can/emergency";
export const CAN_EMERGENCY_STOP_MSG_TYPE: TypeClass<keyof MessagesMap> = "can_msgs/msg/Emergency";

export const NOTIFY_PATH_TOPIC: string = "/rms/ktp/task/notify/path";
export const NOTIFY_PATH_MSG_TYPE: TypeClass<keyof MessagesMap> = "route_msgs/msg/Path";

export const CMD_VEL_TOPIC: string = "/cmd_vel";
export const CMD_VEL_MSG_TYPE: TypeClass<keyof MessagesMap> = "geometry_msgs/msg/Twist";