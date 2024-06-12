import * as fs from "fs";
import * as ini from "ini";
import * as os from "os";
import * as path from "path";
import * as rclnodejs from "rclnodejs";
import { Node, QoS, Subscription, route_msgs } from "rclnodejs";
import * as configFilePathsJSON from "../../common/config/configFilePaths.json";
import { NOTIFY_PATH_MSG_TYPE, NOTIFY_PATH_TOPIC, ROUTE_PATH_TOPIC } from "../../route/domain/route.constants";
import { rtmDataProcessCallback } from "../../common/application/rqtt.callbacks";
import { PATH_TOPIC } from "../domain/path.domain";


export default class PathService {

    private _node: Node;
    private _pathMap: any;
    private _pathMapFileName: string;
    private _notifyPathSubscription: Subscription;

    constructor(node: Node) {
        this._node = node;
        this.bindFunctionContexts.bind(this);
        this.bindFunctionContexts();
        this._pathMap = this.loadMap();
        this._pathMapFileName = "";

        this._notifyPathSubscription = this._node.createSubscription(
            NOTIFY_PATH_MSG_TYPE,
            NOTIFY_PATH_TOPIC,
            { qos: QoS.profileSystemDefault },
            this.notifyPathCallback.bind(this)
        );
    }

    private loadMap(): any {
        try {
            const pathFile: string = configFilePathsJSON.map_config_file_path;
            const homeDirectory: string = os.homedir();
            const mapConfigPath: string = path.join(homeDirectory, pathFile);
            this._node.getLogger().info(`Map Config Path: ${mapConfigPath}`);

            let iniData: any;
            try {
                iniData = ini.parse(fs.readFileSync(mapConfigPath, "utf-8"));
            } catch (error) {
                this._node.getLogger().error(`Failed to read config file: ${error}`);
                return;
            }

            const mapFilePath: string = iniData.file_path;
            this._pathMapFileName = iniData.file_name;

            const mapFileFullPath: string = path.join(homeDirectory, mapFilePath, this._pathMapFileName);
            this._node.getLogger().info(`Map Path: ${mapFileFullPath}`);

            try {
                const fileContent = fs.readFileSync(mapFileFullPath, "utf-8");
                const pathData: any = JSON.parse(fileContent);

                return pathData;
            } catch (error) {
                this._node.getLogger().error(`Failed to read map file: ${error}`);
                return;
            }
        } catch (error: any) {
            this._node.getLogger().error(`Failed to read map file: ${error}`);
        }
    }

    private bindFunctionContexts(): void {
        this.loadMap.bind(this);
    }

    private notifyPathCallback(_notifyPath: any): void {
        const nodeArray: Array<route_msgs.msg.Node> = _notifyPath.node_list;
        const nodeArrayLen: number = nodeArray.length;

        const pathJSONArray: Array<any> = [];

        for (const [index, node] of nodeArray.entries()) {
            let pathJSON: any = {};

            pathJSON.nodeId = node.node_id;
            pathJSON.position = node.position;
            pathJSON.type = node.type;
            pathJSON.kind = node.kind;
            pathJSON.heading = node.heading;
            pathJSON.direction = node.direction;
            pathJSON.drivingOption = node.driving_option;

            const detectionRange: Array<any> = [];

            for (const dr of node.detection_range) {
                let detecionRangeItem: any = {};
                detecionRangeItem.actionCode = dr.action_code;
                detecionRangeItem.widthLeft = dr.width_left;
                detecionRangeItem.widthRigth = dr.width_right;
                detecionRangeItem.height = dr.height;
                detecionRangeItem.offset = dr.offset;
                detectionRange.push(detecionRangeItem);
            }

            pathJSON.detectionRange = detectionRange;

            pathJSONArray.push(pathJSON);

            if (index === nodeArrayLen - 1) {
                break;
            }
        }
        rtmDataProcessCallback(PATH_TOPIC, pathJSONArray);
    }
}