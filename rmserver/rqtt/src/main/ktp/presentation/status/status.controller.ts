import { Node } from "rclnodejs";
import StatusService from "../../application/status/status.service";

export default class StatusController {

    private _statusService: StatusService;

    constructor(node: Node) {
        this._statusService = new StatusService(node);
    }
}