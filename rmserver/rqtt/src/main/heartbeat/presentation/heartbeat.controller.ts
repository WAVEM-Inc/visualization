import mqtt from "mqtt/*";
import HeartBeatService from "../application/heartbeat.service";
import Rqtt from "../../common/application/rqtt";
import { Node } from "rclnodejs";
import { MTR_TOPIC_FORMAT } from "../../common/domain/common.constants";
import { HEARTBEAT_TOPIC } from "../domain/heartbeat.constants";

export default class HeartBeatController {

    private _rqtt: Rqtt;
    private _rqttC: mqtt.MqttClient;
    private _heartBeatService: HeartBeatService;

    constructor(rqttC: mqtt.MqttClient, node: Node) {
        this._rqtt = new Rqtt(node);
        this._rqttC = rqttC;
        this._heartBeatService = new HeartBeatService();

        this.heartBeat();
    }

    private heartBeat(): void {
        const heartBeatTopic: string = `${MTR_TOPIC_FORMAT}${HEARTBEAT_TOPIC}`;
        this._rqtt.subscribe(this._rqttC, heartBeatTopic);
        this._rqtt.addSubscriptionCallback(this._rqttC, heartBeatTopic, this._heartBeatService.heartBeatCallback);
    }
}