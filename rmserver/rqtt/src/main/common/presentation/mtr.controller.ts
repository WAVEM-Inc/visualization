import mqtt from "mqtt/*";
import { Node } from "rclnodejs";
import HeartBeatController from "../../heartbeat/presentation/heartbeat.controller";
import TaskController from "../../ktp/presentation/task/task.controller";
import RouteController from "../../route/presentation/route.controller";

export default class MtRController {

    private _rqttC: mqtt.MqttClient;

    constructor(rqttC: mqtt.MqttClient, node: Node) {
        this._rqttC = rqttC;

        const routeController: RouteController = new RouteController(this._rqttC, node);
        const heartBeatController: HeartBeatController = new HeartBeatController(this._rqttC, node);
        const taskController: TaskController = new TaskController(this._rqttC, node);
    }
}