import mqtt from "mqtt/*";
import HeartBeatService from "../application/heartbeat.service";
import Rqtt from "../../common/application/rqtt";
import { Node } from "rclnodejs";
import { MTR_TOPIC_FORMAT } from "../../common/domain/common.constants";
import { HEARTBEAT_TOPIC } from "../domain/heartbeat.constants";

export default class HeartBeatController {

    private rqtt: Rqtt;
    private rqttC: mqtt.MqttClient;
    private heartBeatService: HeartBeatService;

    constructor(rqttC: mqtt.MqttClient, node: Node) {
        this.rqtt = new Rqtt(node);
        this.rqttC = rqttC;
        this.heartBeatService = new HeartBeatService();

        this.heartBeat();
    }

    private heartBeat(): void {
        const heartBeatTopic: string = `${MTR_TOPIC_FORMAT}${HEARTBEAT_TOPIC}`;
        this.rqtt.subscribe(this.rqttC, heartBeatTopic);
        this.rqtt.addSubscriptionCallback(this.rqttC, heartBeatTopic, this.heartBeatService.heartBeatCallback);
    }
}