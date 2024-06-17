import mqtt from "mqtt/*";
import Rqtt from "../../common/application/rqtt";
import { Node } from "rclnodejs";


export default class PathController {

    private _rqtt: Rqtt;
    private _rqttC: mqtt.MqttClient;

    constructor(rqttC: mqtt.MqttClient, node: Node) {
        this._rqtt = new Rqtt(node);
        this._rqttC = rqttC;
    }
}