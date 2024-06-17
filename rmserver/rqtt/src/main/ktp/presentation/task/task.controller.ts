import mqtt from "mqtt/*";
import { Node } from "rclnodejs";
import Rqtt from "../../../common/application/rqtt";
import { MTR_TOPIC_FORMAT } from "../../../common/domain/common.constants";
import TaskService from "../../application/task/task.service";
import { ASSIGN_TASK_TOPIC } from "../../domain/task/task.constants";

export default class TaskController {

    private _rqtt: Rqtt;
    private _rqttC: mqtt.MqttClient;
    private _taskService: TaskService;

    constructor(rqttC: mqtt.MqttClient, node: Node) {
        this._rqtt = new Rqtt(node);
        this._rqttC = rqttC;
        this._taskService = new TaskService(node);
        
        this.assignTask();
    }

    private assignTask(): void {
        const assignTaskRqttTopic: string = `${MTR_TOPIC_FORMAT}${ASSIGN_TASK_TOPIC}`;
        this._rqtt.subscribe(this._rqttC, assignTaskRqttTopic);
        this._rqtt.addSubscriptionCallback(this._rqttC, assignTaskRqttTopic, this._taskService.assignTaskCallback);
    }
}