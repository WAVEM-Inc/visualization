import React from "react";
import MqttClient from "../../api/mqttClient";
import * as callJSON from "../../assets/json/bluespace/secondary/call.json";
import * as deliveryJSON from "../../assets/json/bluespace/secondary/delivery.json";
import * as rotationTestJSON from "../../assets/json/bluespace/secondary/rotation_test.json";
import * as straightJSON from "../../assets/json/bluespace/secondary/straight.json";
import * as straightTestHorizonJSON from "../../assets/json/bluespace/secondary/straight_horizon_test.json";
import * as straightTestVerticalJSON from "../../assets/json/bluespace/secondary/straight_vertical_test.json";
import * as waitingJSON from "../../assets/json/bluespace/secondary/waiting.json";
import * as emergencyResumeJSON from "../../assets/json/common/emergency_resume.json";
import * as emergencyStopJSON from "../../assets/json/common/emergency_stop.json";
import * as controlGrapySyncJSON from "../../assets/json/control_graphsync.json";
import * as controlMoveToDestJSON from "../../assets/json/control_movetodest.json";
import * as controlMsCompleteJSON from "../../assets/json/control_mscomplete.json";
import * as missionJSON from "../../assets/json/mission.json";
import BlueSpaceRequestComponent from "../../components/bluespace/BlueSpaceRequestComponent";
import MapComponent from "../../components/map/MapComponents";
import TopComponent from "../../components/top/TopComponent";
import { MapState } from "../../domain/map/MapDomain";
import { TopState } from "../../domain/top/TopDomain";
import { onClickMqttPublish } from "../../utils/Utils";
import "./BlueSpaceDashBoardPage.css";

interface BlueSpaceDashBoardPageProps {
    mqttClient: MqttClient;
    topState: TopState;
    mapState: MapState;
}

const BlueSpaceDashBoardPage: React.FC<BlueSpaceDashBoardPageProps> = ({
    mqttClient,
    topState,
    mapState
}: BlueSpaceDashBoardPageProps): React.ReactElement<any, any> | null => {
    const blueSpaceCoord: naver.maps.LatLng = new naver.maps.LatLng(37.305985, 127.2401652);
    const requestTopicFormat: string = "/rms/ktp/dummy/request";
    const requestRouteToPoseTopic: string = `${requestTopicFormat}/route_to_pose`;
    const requestEmergencyTopic: string = `${requestTopicFormat}/can/emergency`;
    const requestGoalCancelTopic: string = `${requestTopicFormat}/goal/cancel`;
    const requestInitTopic: string = `${requestTopicFormat}/can/init`;
    const requestTaskTopic: string =`${requestTopicFormat}/task`;
    const requestPathRenewTopic: string = `${requestTopicFormat}/path/renew`;

    const onStraightClick = (): void => {
        onClickMqttPublish(mqttClient!, requestRouteToPoseTopic, straightJSON);
    }

    const onStraightHorizonTestClick = (): void => {
        onClickMqttPublish(mqttClient!, requestRouteToPoseTopic, straightTestHorizonJSON);
    }

    const onStraightVerticalTestClick = (): void => {
        onClickMqttPublish(mqttClient!, requestRouteToPoseTopic, straightTestVerticalJSON);
    }

    const onRotationTestClick = (): void => {
        onClickMqttPublish(mqttClient!, requestRouteToPoseTopic, rotationTestJSON);
    }

    const onCallClick = (): void => {
        onClickMqttPublish(mqttClient!, requestRouteToPoseTopic, callJSON);
    }

    const onDeliveryClick = (): void => {
        onClickMqttPublish(mqttClient!, requestRouteToPoseTopic, deliveryJSON);
    }

    const onWaitingClick = (): void => {
        onClickMqttPublish(mqttClient!, requestRouteToPoseTopic, waitingJSON);
    }

    const onCanInitClick = (): void => {
        onClickMqttPublish(mqttClient!, requestInitTopic, {
            "can_sign_tran_state": true
        });
    }

    const onPathRenewClick = (): void => {
        onClickMqttPublish(mqttClient!, requestPathRenewTopic, {});
    }

    const onEmergencyStopClick = (): void => {
        onClickMqttPublish(mqttClient!, requestEmergencyTopic, emergencyStopJSON);
    }

    const onEmergencyResumeClick = (): void => {
        onClickMqttPublish(mqttClient!, requestEmergencyTopic, emergencyResumeJSON);
    }

    const onGoalCancelClick = (): void => {
        onClickMqttPublish(mqttClient!, requestGoalCancelTopic, {
            "cancel": true
        }); 
        mapState.path = null;
        mapState.routeStatus = null;
    }

    const onMissionClick = (): void => {
        onClickMqttPublish(mqttClient!, requestTaskTopic, missionJSON);
    }

    const onControlMoveToDestClick = (): void => {
        onClickMqttPublish(mqttClient!, requestTaskTopic, controlMoveToDestJSON);
    }

    const onControlMsCompleteClick = (): void => {
        onClickMqttPublish(mqttClient!, requestTaskTopic, controlMsCompleteJSON);
    }

    const onControlGraphSyncClick = (): void => {
        onClickMqttPublish(mqttClient!, requestTaskTopic, controlGrapySyncJSON);
    }

    return (
        <div className="dash_board_container">
            <div className="top_component_container">
                <TopComponent
                    state={topState}
                 />
            </div>
            <div className="map_component_container">
                <MapComponent
                    state={mapState}
                    center={blueSpaceCoord}
                    onCanInitClick={onCanInitClick}
                    onPathRenewClick={onPathRenewClick}
                    onEmergencyStopClick={onEmergencyStopClick}
                    onEmergencyResumeClick={onEmergencyResumeClick}
                    onGoalCancelClick={onGoalCancelClick}
                />
            </div>
            <div className="request_component_container">
                <BlueSpaceRequestComponent
                    onStraightClick={onStraightClick}
                    onStraightHorizonTestClick={onStraightHorizonTestClick}
                    onStraightVerticalTestClick={onStraightVerticalTestClick}
                    onRotationTestClick={onRotationTestClick}
                    onCallClick={onCallClick}
                    onDeliveryClick={onDeliveryClick}
                    onWaitingClick={onWaitingClick}
                    onMissionClick={onMissionClick}
                    onControlMoveToDestClick={onControlMoveToDestClick}
                    onControlMsCompleteClick={onControlMsCompleteClick}
                    onControlGraphSyncClick={onControlGraphSyncClick}
                />
            </div>
        </div>
    );
}

export default BlueSpaceDashBoardPage;