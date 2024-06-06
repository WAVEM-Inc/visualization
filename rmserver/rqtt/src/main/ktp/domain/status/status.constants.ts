import { MessagesMap, TypeClass } from "rclnodejs";

export const RBT_STATUS_TOPIC: string = "/rms/ktp/data/rbt_status";
export const RBT_STATUS_MSG_TYPE: TypeClass<keyof MessagesMap> = "ktp_data_msgs/msg/Status";

export const SERVICE_STATUS_TOPIC: string = "/rms/ktp/data/service_status";
export const SERVICE_STATUS_MSG_TYPE: TypeClass<keyof MessagesMap> = "ktp_data_msgs/msg/ServiceStatus";