import { MessagesMap, TypeClass } from "rclnodejs";

export const ERROR_REPORT_TOPIC: string = "/rms/ktp/data/error_report";
export const ERROR_REPORT_MSG_TYPE: TypeClass<keyof MessagesMap> = "ktp_data_msgs/msg/ErrorReport";

export const CONTROL_REPORT_TOPIC: string = "/rms/ktp/data/control_report";
export const CONTROL_REPORT_MSG_TYPE: TypeClass<keyof MessagesMap> = "ktp_data_msgs/msg/ControlReport";