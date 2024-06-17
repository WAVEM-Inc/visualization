import { Node } from "rclnodejs";
import ReportService from "../../application/report/report.service";

export default class ReportController {

    private _reportService: ReportService;

    constructor(node: Node) {
        this._reportService = new ReportService(node);
    }
}