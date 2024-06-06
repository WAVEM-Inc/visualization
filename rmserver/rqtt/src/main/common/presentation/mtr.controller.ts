import mqtt from "mqtt/*";
import { Node } from "rclnodejs";
import HeartBeatController from "../../heartbeat/presentation/heartbeat.controller";
import TaskController from "../../ktp/presentation/task/task.controller";
import RouteController from "../../route/presentation/route.controller";

export default class MtRController {

    private rqttC: mqtt.MqttClient;

    constructor(rqttC: mqtt.MqttClient, node: Node) {
        this.rqttC = rqttC;

        const routeController: RouteController = new RouteController(this.rqttC, node);
        const heartBeatController: HeartBeatController = new HeartBeatController(this.rqttC, node);
        const taskController: TaskController = new TaskController(this.rqttC, node);
    }
}