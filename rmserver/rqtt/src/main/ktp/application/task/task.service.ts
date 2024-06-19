import * as rclnodejs from "rclnodejs";
import { Client, Node, QoS, ktp_data_msgs } from "rclnodejs";
import { ASSIGN_CONOTROL_RESOURCE_ID, ASSIGN_CONTROL_SERVICE, ASSIGN_CONTROL_SERVICE_TYPE, ASSIGN_DETECTED_OBJECT_RESOURCE_ID, ASSIGN_MISSION_RESOURCE_ID, ASSIGN_MISSION_SERVICE, ASSIGN_MISSION_SERVICE_TYPE, ASSIGN_TASK_TOPIC } from "../../domain/task/task.constants";

export default class TaskService {

    private _node: Node;
    private _assignControlClient: Client<"ktp_data_msgs/srv/AssignControl">;
    private _assignMissionClient: Client<"ktp_data_msgs/srv/AssignMission">;

    constructor(node: Node) {
        this._node = node;
        this.bindFunctionContexts.bind(this);
        this.bindFunctionContexts();

        this._assignControlClient = this._node.createClient(
            ASSIGN_CONTROL_SERVICE_TYPE,
            ASSIGN_CONTROL_SERVICE,
            { qos: QoS.profileServicesDefault }
        );

        this._assignMissionClient = this._node.createClient(
            ASSIGN_MISSION_SERVICE_TYPE,
            ASSIGN_MISSION_SERVICE,
            { qos: QoS.profileServicesDefault }
        );
    }

    private bindFunctionContexts(): void {
        this.assignTaskCallback = this.assignTaskCallback.bind(this);
    }

    public assignTaskCallback(message: any): void {
        this._node.getLogger().info(`Assign Task message : ${JSON.stringify(message)}`);

        try {
            const resourceId: string = message.id;
            const resourceData: any = message.data;

            switch (resourceId) {
                case ASSIGN_CONOTROL_RESOURCE_ID:
                    this.assignControlServiceRequest(resourceData);
                    break;
                case ASSIGN_MISSION_RESOURCE_ID:
                    this.assignMissionServiceReqeust(resourceData);
                    break;
                case ASSIGN_DETECTED_OBJECT_RESOURCE_ID:
                    break;
                default:
                    break;
            }
        } catch (error: any) {
            this._node.getLogger().error(`${ASSIGN_TASK_TOPIC} : ${error}`)
        }
    }

    private assignControlServiceRequest(control: any): void {
        let _control: any = {
            request_time: "",
            control_id: "",
            owner: "",
            control_code: "",
            control_data: {
                mission_id: "",
                map_id: "",
                request_id: "",
                is_return: false
            }
        };

        _control.request_time = control.request_time;
        _control.control_id = control.control_id;
        _control.owner = control.owner;
        _control.control_code = control.control_code;
        _control.control_data.mission_id = control.control_data.mission_id;
        _control.control_data.map_id = control.control_data.map_id;
        _control.control_data.request_id = control.control_data.request_id;
        this._node.getLogger().info(`${ASSIGN_CONTROL_SERVICE} ============== Return Control : ${control.control_data.return} ==============`);

        if (control.control_data.return === true) {
            _control.control_data.is_return = true;
        } else {
            _control.control_data.is_return = false;
        }

        const assignControlRequest: ktp_data_msgs.srv.AssignControl_Request = rclnodejs.createMessageObject("ktp_data_msgs/srv/AssignControl_Request");
        assignControlRequest.control = _control;
        this._node.getLogger().info(`${ASSIGN_CONTROL_SERVICE} control request : ${JSON.stringify(assignControlRequest)}`);

        try {
            this._assignControlClient.waitForService(1000)
                .then((result: boolean) => {
                    if (!result) {
                        this._node.getLogger().error(`${ASSIGN_CONTROL_SERVICE} is not available`);
                        return;
                    }

                    this._node.getLogger().info(`${ASSIGN_CONTROL_SERVICE} ============== Control : ${JSON.stringify(assignControlRequest)} ==============`);

                    this._assignControlClient.sendRequest(assignControlRequest, (response: ktp_data_msgs.srv.AssignControl_Response) => {
                        this._node.getLogger().info(`${ASSIGN_CONTROL_SERVICE} response : ${JSON.stringify(response)}`);
                    })
                })
                .catch((error: any) => {
                    this._node.getLogger().error(`${ASSIGN_CONTROL_SERVICE} : ${error}`);
                });
        } catch (error: any) {
            this._node.getLogger().error(`${ASSIGN_CONTROL_SERVICE} : ${error}`);
        }
    }

    private assignMissionServiceReqeust(mission: any): void {
        const assignMissionRequest: ktp_data_msgs.srv.AssignMission_Request = rclnodejs.createMessageObject("ktp_data_msgs/srv/AssignMission_Request");
        assignMissionRequest.mission = mission;
        this._node.getLogger().info(`${ASSIGN_MISSION_SERVICE} mission request : ${JSON.stringify(mission)}`);

        try {
            this._assignMissionClient.waitForService(1000)
                .then((result: boolean) => {
                    if (!result) {
                        this._node.getLogger().error(`${ASSIGN_MISSION_SERVICE} is not available`);
                        return;
                    }

                    this._assignMissionClient.sendRequest(assignMissionRequest, (response: ktp_data_msgs.srv.AssignMission_Response) => {
                        this._node.getLogger().info(`${ASSIGN_MISSION_SERVICE} response : ${JSON.stringify(response)}`);
                    });
                })
                .catch((error: any) => {
                    this._node.getLogger().error(`${ASSIGN_MISSION_SERVICE} : ${error}`);
                });
        } catch (error: any) {
            this._node.getLogger().error(`${ASSIGN_MISSION_SERVICE} : ${error}`);
        }
    }
}