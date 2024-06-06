import { Node, QoS, Subscription } from "rclnodejs";
import { rtmDataProcessCallback } from "../../common/application/rqtt.callbacks";
import { BATTERY_STATE_MSG_TYPE, BATTERY_STATE_TOPIC, CMD_VEL_MSG_TYPE, CMD_VEL_TOPIC, ODOM_EULAR_MSG_TYPE, ODOM_EULAR_TOPIC, UBLOX_FIX_MSG_TYPE, UBLOX_FIX_TOPIC } from "../domain/sensor.constants";

export default class SensorService {

    private ubloxFixSubscription: Subscription;
    private odomEularSubscription: Subscription;
    private batteryStateSubscription: Subscription;
    private cmdVelSubscription: Subscription;

    constructor(node: Node) {
        this.ubloxFixSubscription = node.createSubscription<typeof UBLOX_FIX_MSG_TYPE>(
            UBLOX_FIX_MSG_TYPE,
            UBLOX_FIX_TOPIC,
            { qos: QoS.profileSensorData },
            this.ubloxFixCallback.bind(this)
        );

        this.odomEularSubscription = node.createSubscription<typeof ODOM_EULAR_MSG_TYPE>(
            ODOM_EULAR_MSG_TYPE,
            ODOM_EULAR_TOPIC,
            { qos: QoS.profileSensorData },
            this.odomEularCallback.bind(this)
        );

        this.batteryStateSubscription = node.createSubscription<typeof BATTERY_STATE_MSG_TYPE>(
            BATTERY_STATE_MSG_TYPE,
            BATTERY_STATE_TOPIC,
            { qos: QoS.profileSensorData },
            this.batteryStateCallback.bind(this)
        );

        this.cmdVelSubscription = node.createSubscription<typeof CMD_VEL_MSG_TYPE>(
            CMD_VEL_MSG_TYPE,
            CMD_VEL_TOPIC,
            { qos: QoS.profileSystemDefault },
            this.cmdVelCallback.bind(this)
        );
    }

    private ubloxFixCallback(_ubloxFix: any): void {
        rtmDataProcessCallback(this.ubloxFixSubscription.topic, _ubloxFix);
    }

    private odomEularCallback(_odomEular: any): void {
        rtmDataProcessCallback(this.odomEularSubscription.topic, _odomEular);
    }

    private batteryStateCallback(_batteryState: any): void {
        rtmDataProcessCallback(this.batteryStateSubscription.topic, _batteryState);
    }

    private cmdVelCallback(_cmdVel: any): void {
        rtmDataProcessCallback(this.cmdVelSubscription.topic, _cmdVel);
    }
}