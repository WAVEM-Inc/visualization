import mqtt from "mqtt/*";
import { Node } from "rclnodejs";
import Rqtt from "../../common/application/rqtt";
import { MTR_TOPIC_FORMAT } from "../../common/domain/common.constants";
import RouteService from "../application/route.service";
import { CAN_EMERGENCY_STOP_TOPIC, GOAL_CANCEL_TOPIC, PATH_RENEW_TOPIC, PATH_SELECT_TOPIC, ROUTE_TO_POSE_ACTION } from "../domain/route.constants";

export default class RouteController {

    private rqtt: Rqtt;
    private rqttC: mqtt.MqttClient;
    private routeService: RouteService;

    constructor(rqttC: mqtt.MqttClient, node: Node) {
        this.rqtt = new Rqtt(node);
        this.rqttC = rqttC;
        this.routeService = new RouteService(node);

        this.routeToPose();
        this.pathSelect();
        this.pathRenew();
        this.goalCancel();
        this.canEmergencyStop();
    }

    private routeToPose(): void {
        const routeToPoseRqttTopic: string = `${MTR_TOPIC_FORMAT}${ROUTE_TO_POSE_ACTION}`;
        this.rqtt.subscribe(this.rqttC, routeToPoseRqttTopic);
        this.rqtt.addSubscriptionCallback(this.rqttC, routeToPoseRqttTopic, this.routeService.routeToPoseCallback);
    }

    private pathSelect(): void {
        const pathSelectRqttTopic: string = `${MTR_TOPIC_FORMAT}${PATH_SELECT_TOPIC}`;
        this.rqtt.subscribe(this.rqttC, pathSelectRqttTopic);
        this.rqtt.addSubscriptionCallback(this.rqttC, pathSelectRqttTopic, this.routeService.pathSelectCallback);
    }

    private pathRenew(): void {
        const pathRenewRqttTopic: string = `${MTR_TOPIC_FORMAT}${PATH_RENEW_TOPIC}`;
        this.rqtt.subscribe(this.rqttC, pathRenewRqttTopic);
        this.rqtt.addSubscriptionCallback(this.rqttC, pathRenewRqttTopic, this.routeService.pathRenewCallback);
    }

    private goalCancel(): void {
        const goalCancelRqttTopic: string = `${MTR_TOPIC_FORMAT}${GOAL_CANCEL_TOPIC}`;
        this.rqtt.subscribe(this.rqttC, goalCancelRqttTopic);
        this.rqtt.addSubscriptionCallback(this.rqttC, goalCancelRqttTopic, this.routeService.goalCancelCallback);
    }

    private canEmergencyStop(): void {
        const canEmergencyStopRqttTopic: string = `${MTR_TOPIC_FORMAT}${CAN_EMERGENCY_STOP_TOPIC}`;
        this.rqtt.subscribe(this.rqttC, canEmergencyStopRqttTopic);
        this.rqtt.addSubscriptionCallback(this.rqttC, canEmergencyStopRqttTopic, this.routeService.canEmergencyStopCallback);
    }
}