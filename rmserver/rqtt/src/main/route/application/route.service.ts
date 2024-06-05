import * as fs from "fs";
import * as ini from "ini";
import * as os from "os";
import * as path from "path";
import * as rclnodejs from "rclnodejs";
import { ActionClient, Node, Publisher, QoS, route_msgs, action_msgs } from "rclnodejs";
import { rtmDataProcessCallback } from "../../common/application/rqtt.callbacks";
import * as configFilePathsJSON from "../../common/config/configFilePaths.json";
import { CAN_EMERGENCY_STOP_MSG_TYPE, CAN_EMERGENCY_STOP_TOPIC, GOAL_CANCEL_TOPIC, PATH_RENEW_TOPIC, PATH_SELECT_TOPIC, ROUTE_PATH_TOPIC, ROUTE_STATUS_TOPIC, ROUTE_TO_POSE_ACTION, ROUTE_TO_POSE_ACTION_TYPE } from "../domain/route.constants";

export default class RouteService {

    private node: Node;
    private pathMap: any;
    private pathMapFileName: string;
    private routeToPoseGoalIndex: number;
    private routeToPoseGoalList: Array<route_msgs.action.RouteToPose_Goal>;
    private routeToPoseGoalListSize: number = 0;
    private routeToPoseActionClientGoalHandle: rclnodejs.ClientGoalHandle<"route_msgs/action/RouteToPose"> | null;
    private routeToPoseActionClient: ActionClient<"route_msgs/action/RouteToPose">;
    private canEmergencyStopPublisher: Publisher<"can_msgs/msg/Emergency">;

    constructor(node: Node) {
        this.node = node;
        this.bindFunctionContexts.bind(this);
        this.bindFunctionContexts();
        this.pathMap = this.loadMap();
        this.pathMapFileName = "";

        this.routeToPoseGoalIndex = 0;
        this.routeToPoseGoalList = [];
        this.routeToPoseGoalListSize = 0;
        this.routeToPoseActionClientGoalHandle = null;

        this.routeToPoseActionClient = new rclnodejs.ActionClient(
            this.node,
            ROUTE_TO_POSE_ACTION_TYPE,
            ROUTE_TO_POSE_ACTION
        );

        this.canEmergencyStopPublisher = this.node.createPublisher(
            CAN_EMERGENCY_STOP_MSG_TYPE,
            CAN_EMERGENCY_STOP_TOPIC,
            { qos: QoS.profileSystemDefault }
        );
    }

    private bindFunctionContexts(): void {
        this.routeToPoseCallback = this.routeToPoseCallback.bind(this);
        this.canEmergencyStopCallback = this.canEmergencyStopCallback.bind(this);
        this.pathSelectCallback = this.pathSelectCallback.bind(this);
        this.pathRenewCallback = this.pathRenewCallback.bind(this);
        this.goalCancelCallback = this.goalCancelCallback.bind(this);
        this.loadMap.bind(this);
    }

    private loadMap(): any {
        try {
            const pathFile: string = configFilePathsJSON.map_config_file_path;
            const homeDirectory: string = os.homedir();
            const mapConfigPath: string = path.join(homeDirectory, pathFile);
            this.node.getLogger().info(`Map Config Path: ${mapConfigPath}`);

            let iniData: any;
            try {
                iniData = ini.parse(fs.readFileSync(mapConfigPath, 'utf-8'));
            } catch (error) {
                this.node.getLogger().error(`Failed to read config file: ${error}`);
                return;
            }

            const mapFilePath: string = iniData.file_path;
            this.pathMapFileName = iniData.file_name;

            const mapFileFullPath: string = path.join(homeDirectory, mapFilePath, this.pathMapFileName);
            this.node.getLogger().info(`Map Path: ${mapFileFullPath}`);

            try {
                const fileContent = fs.readFileSync(mapFileFullPath, "utf-8");
                const pathData: any = JSON.parse(fileContent);

                return pathData;
            } catch (error) {
                this.node.getLogger().error(`Failed to read map file: ${error}`);
                return;
            }
        } catch (error: any) {
            this.node.getLogger().error(`Failed to read map file: ${error}`);
        }
    }

    public routeToPoseCallback(message: any): void {
        this.node.getLogger().info(`${ROUTE_TO_POSE_ACTION} message : ${JSON.stringify(message)}`);

        try {
            const pathMapArray: Array<any> = this.pathMap.path;

            if (this.routeToPoseGoalListSize === 0) {
                const pathId: string = message.path.id;
                const pathName: string = message.path.name;
                this.node.getLogger().info(`${ROUTE_TO_POSE_ACTION} pathId : ${pathId}, pathName : ${pathName}`);

                let nodeArray: Array<any> = [];

                for (const [pathIndex, pathMap] of pathMapArray.entries()) {
                    if (pathMap.id === pathId) {
                        this.node.getLogger().info(`${ROUTE_TO_POSE_ACTION} path found: ${pathMap.id}`);
                        nodeArray = pathMap.nodeList;
                        this.routeToPoseGoalListSize = nodeArray.length - 1;
                        break;
                    }

                    if (pathIndex === pathMapArray.length - 1) {
                        this.node.getLogger().error(`${ROUTE_TO_POSE_ACTION} path not found`);
                        break;
                    }
                }

                this.node.getLogger().info(`${ROUTE_TO_POSE_ACTION} routeToPoseGoalListSize : ${this.routeToPoseGoalListSize}`);

                for (let i = 0; i < nodeArray.length - 1; i++) {
                    const goal: route_msgs.action.RouteToPose_Goal = rclnodejs.createMessageObject("route_msgs/action/RouteToPose_Goal");
                    const startNode: route_msgs.msg.Node = rclnodejs.createMessageObject("route_msgs/msg/Node");
                    const endNode: route_msgs.msg.Node = rclnodejs.createMessageObject("route_msgs/msg/Node");

                    if (i === this.routeToPoseGoalListSize) {
                        this.node.getLogger().info(`${ROUTE_TO_POSE_ACTION} converting node to goal finished...`);
                        this.node.getLogger().info(`${ROUTE_TO_POSE_ACTION} routeToPoseGoalListSize : ${this.routeToPoseGoalListSize}`);
                        break;
                    }

                    const pathStartNode: any = nodeArray[i];

                    startNode.node_id = pathStartNode.nodeId.split("-")[2];
                    startNode.position = pathStartNode.position;
                    startNode.type = pathStartNode.type;
                    startNode.kind = pathStartNode.kind;
                    startNode.heading = parseFloat(pathStartNode.heading);
                    startNode.direction = pathStartNode.direction;
                    startNode.driving_option = pathStartNode.drivingOption;

                    if (pathStartNode.detectionRange.length != 0) {
                        startNode.detection_range = this.extractDetectionRange(pathStartNode.detectionRange);
                    } else {
                        startNode.detection_range = [];
                    }

                    const pathEndNode: any = nodeArray[i + 1];

                    endNode.node_id = pathEndNode.nodeId.split("-")[2];
                    endNode.position = pathEndNode.position;
                    endNode.type = pathEndNode.type;
                    endNode.kind = pathEndNode.kind;
                    endNode.heading = parseFloat(pathEndNode.heading);
                    endNode.direction = pathEndNode.direction;
                    endNode.driving_option = pathEndNode.drivingOption;

                    if (pathEndNode.detectionRange.length != 0) {
                        endNode.detection_range = this.extractDetectionRange(pathEndNode.detectionRange);
                    } else {
                        endNode.detection_range = [];
                    }

                    goal.start_node = startNode;
                    goal.end_node = endNode;
                    this.routeToPoseGoalList.push(goal);
                }

                rtmDataProcessCallback(ROUTE_PATH_TOPIC, nodeArray);

                if (message.isEnableToCommandRoute === false) {
                    this.node.getLogger().error(`${ROUTE_TO_POSE_ACTION} cannot command route`);
                    this.routeToPoseFlushGoal();
                    return;
                } else {
                    this.routeToPoseSendGoal();
                }
            } else {
                this.node.getLogger().error(`${ROUTE_TO_POSE_ACTION} navigation is already proceeding...`);
                rtmDataProcessCallback(ROUTE_STATUS_TOPIC, {
                    is_driving: false,
                    status_code: 4
                });
            }
        } catch (error: any) {
            this.node.getLogger().error(`${ROUTE_TO_POSE_ACTION} : ${error.message}`);
        }
    }

    private extractDetectionRange(detectionRange: Array<any>): Array<route_msgs.msg.DetectionRange> {
        const detectionRangeArray: Array<route_msgs.msg.DetectionRange> = [];

        for (const dr of detectionRange) {
            const detectionRange: route_msgs.msg.DetectionRange = rclnodejs.createMessageObject("route_msgs/msg/DetectionRange");
            detectionRange.offset = dr.offset;
            detectionRange.width_left = dr.widthLeft;
            detectionRange.width_right = dr.widthRigth;
            detectionRange.height = dr.height;
            detectionRange.action_code = dr.actionCode;
            detectionRangeArray.push(dr);
        }

        return detectionRangeArray;
    }

    private routeToPoseFlushGoal(): void {
        this.routeToPoseGoalIndex = 0;
        this.routeToPoseGoalList = [];
        this.routeToPoseGoalListSize = 0;
        this.node.getLogger().info("============== Goal Flush ==============");
    }

    private async routeToPoseSendGoal(): Promise<void> {
        try {
            const goal: route_msgs.action.RouteToPose_Goal = this.routeToPoseGoalList[this.routeToPoseGoalIndex];

            this.node.getLogger().info(`${ROUTE_TO_POSE_ACTION} Send Goal [${this.routeToPoseGoalIndex} / ${this.routeToPoseGoalListSize}]`);

            const isRouteToPoseServerReady: boolean = await this.routeToPoseActionClient.waitForServer(0.75);

            if (!isRouteToPoseServerReady) {
                this.node.getLogger().error(`${ROUTE_TO_POSE_ACTION} server is not ready...`);
                rtmDataProcessCallback(ROUTE_STATUS_TOPIC, {
                    is_driving: false,
                    status_code: 3
                });
                this.routeToPoseFlushGoal();
                return;
            }

            this.routeToPoseActionClientGoalHandle = await this.routeToPoseActionClient.sendGoal(goal, (feedback) => {
                this.routeToPoseFeedbackCallback(feedback);
            });

            if (!this.routeToPoseActionClientGoalHandle) {
                this.node.getLogger().error(`${ROUTE_TO_POSE_ACTION} goal rejected`);
                return;
            }

            this.node.getLogger().info(`${ROUTE_TO_POSE_ACTION} goal accepted`);

            const routeToPoseGoalResult: route_msgs.action.RouteToPose_Result = await this.routeToPoseActionClientGoalHandle.getResult();

            if (this.routeToPoseActionClientGoalHandle.isSucceeded()) {
                this.routeToPoseResultCallback(routeToPoseGoalResult);
            } else {
                this.node.getLogger().error(`${ROUTE_TO_POSE_ACTION} goal handle failed`);
                return;
            }
        } catch (error: any) {
            this.node.getLogger().error(`${ROUTE_TO_POSE_ACTION} Send Goal : ${error}`);
            rtmDataProcessCallback(ROUTE_STATUS_TOPIC, {
                is_driving: false,
                status_code: 3
            });
            return;
        }
    }

    private routeToPoseFeedbackCallback(feedback: route_msgs.action.RouteToPose_Feedback): void {
        this.node.getLogger().info(`${ROUTE_TO_POSE_ACTION} Received feedback: ${JSON.stringify(feedback)}`);

        const status_code: number = feedback.status_code;
        const currentGoal: route_msgs.action.RouteToPose_Goal = this.routeToPoseGoalList[this.routeToPoseGoalIndex];

        if (status_code === 1001) {
            rtmDataProcessCallback(ROUTE_STATUS_TOPIC, {
                is_driving: true,
                status_code: 0,
                node_index: this.routeToPoseGoalIndex,
                node_info: [currentGoal.start_node.node_id, currentGoal.end_node.node_id]
            });
        } else return;
    }

    private routeToPoseResultCallback(result: route_msgs.action.RouteToPose_Result): void {
        this.node.getLogger().info(`${ROUTE_TO_POSE_ACTION} Received result : ${JSON.stringify(result)}`);

        const status: number = result.result;

        if (status === 1001) {
            const isGoalListFinished: boolean = this.routeToPoseGoalIndex === this.routeToPoseGoalListSize - 1;
            const currentGoal: route_msgs.action.RouteToPose_Goal = this.routeToPoseGoalList[this.routeToPoseGoalIndex];

            if (isGoalListFinished) {
                this.node.getLogger().info(`${ROUTE_TO_POSE_ACTION} navigation finished...`);
                rtmDataProcessCallback(ROUTE_STATUS_TOPIC, {
                    is_driving: true,
                    status_code: 2,
                    node_index: this.routeToPoseGoalIndex,
                    node_info: [currentGoal.start_node.node_id, currentGoal.end_node.node_id]
                });
                this.routeToPoseFlushGoal();
            } else {
                rtmDataProcessCallback(ROUTE_STATUS_TOPIC, {
                    is_driving: true,
                    status_code: 1,
                    node_index: this.routeToPoseGoalIndex,
                    node_info: [currentGoal.start_node.node_id, currentGoal.end_node.node_id]
                });
                this.routeToPoseGoalIndex++;
                this.node.getLogger().info(`${ROUTE_TO_POSE_ACTION} It will proceed Next Goal [${this.routeToPoseGoalIndex}]`);
                this.routeToPoseSendGoal();
            }
        }
    }

    public pathSelectCallback(message: any): void {
        try {
            this.loadMap();

            const responseJSON: any = {
                current_map_file: this.pathMapFileName,
                paths: this.pathMap
            }

            rtmDataProcessCallback(PATH_SELECT_TOPIC, responseJSON);
            this.node.getLogger().info("=============================== Path Selected ===============================");
        } catch (error: any) {
            this.node.getLogger().error(`${PATH_SELECT_TOPIC} : ${error.message}`);
        }
    }

    public pathRenewCallback(message: any): void {
        try {
            this.loadMap();
            this.node.getLogger().info("=============================== Path Renewed ===============================");
        } catch (error: any) {
            this.node.getLogger().error(`${PATH_RENEW_TOPIC} : ${error.message}`);
        }
    }

    public async goalCancelCallback(message: any): Promise<void> {
        try {
            if (message.cancel === true) {
                const cancelResponse: action_msgs.srv.CancelGoal_Response | undefined = await this.routeToPoseActionClientGoalHandle?.cancelGoal();
                
                if (cancelResponse?.goals_canceling.length! > 0) {
                    this.node.getLogger().info(`${GOAL_CANCEL_TOPIC} goal canceled`);
                } else {
                    this.node.getLogger().error(`${GOAL_CANCEL_TOPIC} failed to goal canceling`);
                }
            }
        } catch (error: any) {
            this.node.getLogger().error(`${GOAL_CANCEL_TOPIC} : ${error.message}`);
        }
    }

    public canEmergencyStopCallback(message: any): void {
        this.canEmergencyStopPublisher.publish(message);
    }
}