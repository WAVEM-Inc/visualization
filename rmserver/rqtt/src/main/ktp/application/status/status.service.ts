import { Node, QoS, Subscription } from "rclnodejs";
import { rtmDataProcessCallback } from "../../../common/application/rqtt.callbacks";
import { RBT_STATUS_MSG_TYPE, RBT_STATUS_TOPIC, SERVICE_STATUS_MSG_TYPE, SERVICE_STATUS_TOPIC } from "../../domain/status/status.constants";

export default class StatusService {

    private _rbtStatusSubscription: Subscription;
    private _serviceStatusSubscription: Subscription;

    constructor(node: Node) {
        this._rbtStatusSubscription = node.createSubscription(
            RBT_STATUS_MSG_TYPE,
            RBT_STATUS_TOPIC,
            { qos: QoS.profileSystemDefault },
            this.rbtStatusCallback.bind(this)
        );

        this._serviceStatusSubscription = node.createSubscription(
            SERVICE_STATUS_MSG_TYPE,
            SERVICE_STATUS_TOPIC,
            { qos: QoS.profileSystemDefault },
            this.serviceStatusCallback.bind(this)
        );
    }

    private rbtStatusCallback(_rbtStatus: any): void {
        const rbtStatusRqttTopic: string = `/status/${this._rbtStatusSubscription.topic.split("/")[4]}`;
        rtmDataProcessCallback(rbtStatusRqttTopic, _rbtStatus);
    }

    private serviceStatusCallback(_serviceStatus: any): void {
        const serviceStatusRqttTopic: string = `/status/${this._serviceStatusSubscription.topic.split("/")[4]}`;
        rtmDataProcessCallback(serviceStatusRqttTopic, _serviceStatus);
    }
}