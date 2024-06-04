import mqtt from "mqtt/*";
import { Node } from "rclnodejs";
import Rqtt from "../../common/application/rqtt";
import { setRtmDataProcessCallback } from "../../common/application/rqtt.callbacks";
import SensorService from "../application/sensor.service";


export default class SensorController {
    
    private rqtt: Rqtt;
    private rqttC: mqtt.MqttClient;

    constructor(rqttC: mqtt.MqttClient, node: Node) {
        this.rqtt = new Rqtt(node);
        this.rqttC = rqttC;

        const sensorService: SensorService = new SensorService(node);

        setRtmDataProcessCallback(async (topic: string, data: any) => {
            try {
                this.rqtt.publish(this.rqttC, topic, JSON.stringify(data));
            } catch (error) {
                console.error("Error publishing message:", error);
                return;
            }
        });
    }
}