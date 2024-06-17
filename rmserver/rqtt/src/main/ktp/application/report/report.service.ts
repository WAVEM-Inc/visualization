import { Node, QoS, Subscription } from "rclnodejs";
import { rtmDataProcessCallback } from "../../../common/application/rqtt.callbacks";
import { CONTROL_REPORT_MSG_TYPE, CONTROL_REPORT_TOPIC, ERROR_REPORT_MSG_TYPE, ERROR_REPORT_TOPIC } from "../../domain/report/report.constants";

export default class ReportService {

    private _errorReportSubscription: Subscription;
    private _controlReportSubscription: Subscription;

    constructor(node: Node) {

        this._errorReportSubscription = node.createSubscription(
            ERROR_REPORT_MSG_TYPE,
            ERROR_REPORT_TOPIC,
            { qos: QoS.profileSystemDefault },
            this.errorReportCallback.bind(this)
        );

        this._controlReportSubscription = node.createSubscription(
            CONTROL_REPORT_MSG_TYPE,
            CONTROL_REPORT_TOPIC,
            { qos: QoS.profileSystemDefault },
            this.controlReportCallback.bind(this)
        )
    }

    private errorReportCallback(_errorReport: any): void {
        const errorReportRqttTopic: string = `/report/${this._errorReportSubscription.topic.split("/")[4]}`;
        rtmDataProcessCallback(errorReportRqttTopic, _errorReport);
    }

    private controlReportCallback(_controlReport: any): void {
        const controlReportRqttTopic: string = `/report/${this._controlReportSubscription.topic.split("/")[4]}`;
        rtmDataProcessCallback(controlReportRqttTopic, _controlReport);
    }
}