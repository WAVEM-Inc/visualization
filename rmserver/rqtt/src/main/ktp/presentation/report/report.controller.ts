import { Node } from "rclnodejs";
import ReportService from "../../application/report/report.service";

export default class ReportController {

    private reportService: ReportService;

    constructor(node: Node) {
        this.reportService = new ReportService(node);
    }
}