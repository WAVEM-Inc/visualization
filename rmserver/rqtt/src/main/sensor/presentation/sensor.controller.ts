import mqtt from "mqtt/*";
import { Node } from "rclnodejs";
import Rqtt from "../../common/application/rqtt";
import { setRtmDataProcessCallback } from "../../common/application/rqtt.callbacks";
import SensorService from "../application/sensor.service";
import { RTM_TOPIC_FORMAT } from "../../common/domain/common.constants";


export default class SensorController {
    
    private sensorService: SensorService;

    constructor(node: Node) {
        this.sensorService = new SensorService(node);
    }
}