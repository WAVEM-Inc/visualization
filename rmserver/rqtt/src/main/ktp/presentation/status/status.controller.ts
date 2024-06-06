import { Node } from "rclnodejs";
import StatusService from "../../application/status/status.service";

export default class StatusController {

    private statusService: StatusService;

    constructor(node: Node) {
        this.statusService = new StatusService(node);
    }
}