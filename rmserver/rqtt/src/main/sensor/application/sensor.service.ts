import { Node, QoS, Subscription } from "rclnodejs";
import { rtmDataProcessCallback } from "../../common/application/rqtt.callbacks";
import { BATTERY_STATE_MSG_TYPE, BATTERY_STATE_TOPIC, ODOM_EULAR_MSG_TYPE, ODOM_EULAR_TOPIC, UBLOX_FIX_MSG_TYPE, UBLOX_FIX_TOPIC } from "../domain/sensor.constants";

export default class SensorService {

    constructor(node: Node) {
        const ubloxFixSubscription: Subscription = node.createSubscription<typeof UBLOX_FIX_MSG_TYPE>(
            UBLOX_FIX_MSG_TYPE,
            UBLOX_FIX_TOPIC,
            { qos: QoS.profileSensorData },
            this.ubloxFixCallback.bind(this)
        );

        const odomEularSubscription: Subscription = node.createSubscription<typeof ODOM_EULAR_MSG_TYPE>(
            ODOM_EULAR_MSG_TYPE,
            ODOM_EULAR_TOPIC,
            { qos: QoS.profileSensorData },
            this.odomEularCallback.bind(this)
        );

        const batteryStateSubscription: Subscription = node.createSubscription<typeof BATTERY_STATE_MSG_TYPE>(
            BATTERY_STATE_MSG_TYPE,
            BATTERY_STATE_TOPIC,
            { qos: QoS.profileSensorData },
            this.batteryStateCallback.bind(this)
        );
    }

    private ubloxFixCallback(_ubloxFix: any) {
        rtmDataProcessCallback(UBLOX_FIX_TOPIC, _ubloxFix);
    }

    private odomEularCallback(_odomEular: any) {
        rtmDataProcessCallback(ODOM_EULAR_TOPIC, _odomEular);
    }

    private batteryStateCallback(_batteryState: any) {
        rtmDataProcessCallback(BATTERY_STATE_TOPIC, _batteryState);
    }
}