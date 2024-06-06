import mqtt from "mqtt/*";
import { Node } from "rclnodejs";
import Rqtt from "../../../common/application/rqtt";
import { MTR_TOPIC_FORMAT } from "../../../common/domain/common.constants";
import TaskService from "../../application/task/task.service";
import { ASSIGN_TASK_TOPIC } from "../../domain/task/task.constants";

export default class TaskController {

    private rqtt: Rqtt;
    private rqttC: mqtt.MqttClient;
    private taskService: TaskService;

    constructor(rqttC: mqtt.MqttClient, node: Node) {
        this.rqtt = new Rqtt(node);
        this.rqttC = rqttC;
        this.taskService = new TaskService(node);
        
        this.assignTask();
    }

    private assignTask(): void {
        const assignTaskRqttTopic: string = `${MTR_TOPIC_FORMAT}${ASSIGN_TASK_TOPIC}`;
        this.rqtt.subscribe(this.rqttC, assignTaskRqttTopic);
        this.rqtt.addSubscriptionCallback(this.rqttC, assignTaskRqttTopic, this.taskService.assignTaskCallback);
    }
}