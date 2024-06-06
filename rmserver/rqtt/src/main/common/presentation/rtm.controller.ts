import mqtt from "mqtt/*";
import { Node } from "rclnodejs";
import StatusController from "../../ktp/presentation/status/status.controller";
import SensorController from "../../sensor/presentation/sensor.controller";
import Rqtt from "../application/rqtt";
import { setRtmDataProcessCallback } from "../application/rqtt.callbacks";
import { RTM_TOPIC_FORMAT } from "../domain/common.constants";
import ReportController from "../../ktp/presentation/report/report.controller";

export default class RtMController {

    private rqtt: Rqtt;
    private rqttC: mqtt.MqttClient;

    constructor(rqttC: mqtt.MqttClient, node: Node) {
        this.rqtt = new Rqtt(node);
        this.rqttC = rqttC;

        const sensorController: SensorController = new SensorController(node);
        const statusCotroller: StatusController = new StatusController(node);
        const reportController: ReportController = new ReportController(node);

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