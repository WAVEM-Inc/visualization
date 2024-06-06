import { ServicesMap, TypeClass } from "rclnodejs";

export const ASSIGN_TASK_TOPIC: string = "/task";

export const ASSIGN_CONTROL_SERVICE: string = "/ktp_data_manager/assign/control";
export const ASSIGN_CONTROL_SERVICE_TYPE: TypeClass<keyof ServicesMap> = "ktp_data_msgs/srv/AssignControl";

export const ASSIGN_MISSION_SERVICE: string = "/ktp_data_manager/assign/mission";
export const ASSIGN_MISSION_SERVICE_TYPE: TypeClass<keyof ServicesMap> = "ktp_data_msgs/srv/AssignMission";

export const ASSIGN_CONOTROL_RESOURCE_ID: string = "rbt_control";
export const ASSIGN_MISSION_RESOURCE_ID: string = "rbt_mission";
export const ASSIGN_DETECTED_OBJECT_RESOURCE_ID: string = "rbt_detected_object";