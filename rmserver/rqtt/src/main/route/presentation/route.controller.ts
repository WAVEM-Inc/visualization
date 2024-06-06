import mqtt from "mqtt/*";
import { Node } from "rclnodejs";
import Rqtt from "../../common/application/rqtt";
import { MTR_TOPIC_FORMAT } from "../../common/domain/common.constants";
import RouteService from "../application/route.service";
import { CAN_EMERGENCY_STOP_TOPIC, GOAL_CANCEL_TOPIC, PATH_RENEW_TOPIC, PATH_SELECT_TOPIC, ROUTE_TO_POSE_ACTION } from "../domain/route.constants";

export default class RouteController {

    private _rqtt: Rqtt;
    private _rqttC: mqtt.MqttClient;
    private _routeService: RouteService;

    constructor(rqttC: mqtt.MqttClient, node: Node) {
        this._rqtt = new Rqtt(node);
        this._rqttC = rqttC;
        this._routeService = new RouteService(node);

        this.routeToPose();
        this.pathSelect();
        this.pathRenew();
        this.goalCancel();
        this.canEmergencyStop();
    }

    private routeToPose(): void {
        const routeToPoseRqttTopic: string = `${MTR_TOPIC_FORMAT}${ROUTE_TO_POSE_ACTION}`;
        this._rqtt.subscribe(this._rqttC, routeToPoseRqttTopic);
        this._rqtt.addSubscriptionCallback(this._rqttC, routeToPoseRqttTopic, this._routeService.routeToPoseCallback);
    }

    private pathSelect(): void {
        const pathSelectRqttTopic: string = `${MTR_TOPIC_FORMAT}${PATH_SELECT_TOPIC}`;
        this._rqtt.subscribe(this._rqttC, pathSelectRqttTopic);
        this._rqtt.addSubscriptionCallback(this._rqttC, pathSelectRqttTopic, this._routeService.pathSelectCallback);
    }

    private pathRenew(): void {
        const pathRenewRqttTopic: string = `${MTR_TOPIC_FORMAT}${PATH_RENEW_TOPIC}`;
        this._rqtt.subscribe(this._rqttC, pathRenewRqttTopic);
        this._rqtt.addSubscriptionCallback(this._rqttC, pathRenewRqttTopic, this._routeService.pathRenewCallback);
    }

    private goalCancel(): void {
        const goalCancelRqttTopic: string = `${MTR_TOPIC_FORMAT}${GOAL_CANCEL_TOPIC}`;
        this._rqtt.subscribe(this._rqttC, goalCancelRqttTopic);
        this._rqtt.addSubscriptionCallback(this._rqttC, goalCancelRqttTopic, this._routeService.goalCancelCallback);
    }

    private canEmergencyStop(): void {
        const canEmergencyStopRqttTopic: string = `${MTR_TOPIC_FORMAT}${CAN_EMERGENCY_STOP_TOPIC}`;
        this._rqtt.subscribe(this._rqttC, canEmergencyStopRqttTopic);
        this._rqtt.addSubscriptionCallback(this._rqttC, canEmergencyStopRqttTopic, this._routeService.canEmergencyStopCallback);
    }
}