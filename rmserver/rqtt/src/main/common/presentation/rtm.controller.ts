import mqtt from "mqtt/*";
import Rqtt from "../application/rqtt";
import { setRtmDataProcessCallback } from "../application/rqtt.callbacks";
import { Node } from "rclnodejs";
import { RTM_TOPIC_FORMAT } from "../domain/common.constants";

export default class RtMController {

    private rqtt: Rqtt;
    private rqttC: mqtt.MqttClient;

    constructor(rqttC: mqtt.MqttClient, node: Node) {
        this.rqtt = new Rqtt(node);
        this.rqttC = rqttC;

        setRtmDataProcessCallback(async (topic: string, data: any): Promise<void> => {
            try {
                this.rqtt.publish(this.rqttC, `${RTM_TOPIC_FORMAT}${topic}`, JSON.stringify(data));
            } catch (error) {
                console.error("Error publishing message:", error);
                return;
            }
        });   
    }
}