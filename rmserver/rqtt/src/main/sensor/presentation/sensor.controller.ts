import { Node } from "rclnodejs";
import SensorService from "../application/sensor.service";


export default class SensorController {

    private _sensorService: SensorService;

    constructor(node: Node) {
        this._sensorService = new SensorService(node);
    }
}