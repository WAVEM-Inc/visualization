import React, { useEffect, useState } from "react";
import MqttClient from "../../../api/mqttClient";
import * as callJSON from "../../../assets/json/bluespace/secondary/call.json";
import * as deliveryJSON from "../../../assets/json/bluespace/secondary/delivery.json";
import * as rotationTestJSON from "../../../assets/json/bluespace/secondary/rotation_test.json";
import * as straightJSON from "../../../assets/json/bluespace/secondary/straight.json";
import * as straightTestHorizonJSON from "../../../assets/json/bluespace/secondary/straight_horizon_test.json";
import * as straightTestVerticalJSON from "../../../assets/json/bluespace/secondary/straight_vertical_test.json";
import * as waitingJSON from "../../../assets/json/bluespace/secondary/waiting.json";
import * as controlGrapySyncJSON from "../../../assets/json/control_graphsync.json";
import * as controlMoveToDestJSON from "../../../assets/json/control_movetodest.json";
import * as controlMsCompleteNoReturnJSON from "../../../assets/json/control_mscomplete_no_return.json";
import * as controlMsCompleteReturnJSON from "../../../assets/json/control_mscomplete_return.json";
import * as deliveringMissionJSON from "../../../assets/json/mission_delivering.json";
import * as retunringMissionJSON from "../../../assets/json/mission_returning.json";
import BlueSpaceRequestComponent from "../../../components/bluespace/BlueSpaceRequestComponent";
import MapComponent from "../../../components/map/MapComponents";
import TopComponent from "../../../components/top/TopComponent";
import { MapState } from "../../../domain/map/MapDomain";
import { TopState } from "../../../domain/top/TopDomain";
import { onClickMqttPublish } from "../../../utils/Utils";
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
    const [isEnableToCommandRoute, setIsEnableToCommandRoute] = useState<string | null>(null);
    const blueSpaceCoord: naver.maps.LatLng = new naver.maps.LatLng(37.305985, 127.2401652);
    const requestTopicFormat: string = "/rms/ktp/dummy/request";
    const requestRouteToPoseTopic: string = `${requestTopicFormat}/route_to_pose`;
    const requestTaskTopic: string =`${requestTopicFormat}/task`;

    useEffect(() => {
        setIsEnableToCommandRoute(localStorage.getItem("isEnableToCommandRoute?"));
    }, [localStorage.getItem("isEnableToCommandRoute?")]);

    const onStraightClick = (): void => {
        const j: any =  {
            isEnableToCommandRoute: isEnableToCommandRoute,
            path: straightJSON
        };

        onClickMqttPublish(mqttClient!, requestRouteToPoseTopic, j);
    }

    const onStraightHorizonTestClick = (): void => {
        const j: any =  {
            isEnableToCommandRoute: isEnableToCommandRoute,
            path: straightTestHorizonJSON
        };

        onClickMqttPublish(mqttClient!, requestRouteToPoseTopic, j);
    }

    const onStraightVerticalTestClick = (): void => {
        const j: any =  {
            isEnableToCommandRoute: isEnableToCommandRoute,
            path: straightTestVerticalJSON
        };

        onClickMqttPublish(mqttClient!, requestRouteToPoseTopic, j);
    }

    const onRotationTestClick = (): void => {
        const j: any =  {
            isEnableToCommandRoute: isEnableToCommandRoute,
            path: rotationTestJSON
        };

        onClickMqttPublish(mqttClient!, requestRouteToPoseTopic, j);
    }

    const onCallClick = (): void => {
        const j: any =  {
            isEnableToCommandRoute: isEnableToCommandRoute,
            path: callJSON
        };

        onClickMqttPublish(mqttClient!, requestRouteToPoseTopic, j);
    }

    const onDeliveryClick = (): void => {
        const j: any =  {
            isEnableToCommandRoute: isEnableToCommandRoute,
            path: deliveryJSON
        };

        onClickMqttPublish(mqttClient!, requestRouteToPoseTopic, j);
    }

    const onWaitingClick = (): void => {
        const j: any =  {
            isEnableToCommandRoute: isEnableToCommandRoute,
            path: waitingJSON
        };

        onClickMqttPublish(mqttClient!, requestRouteToPoseTopic, j);
    }

    const onDeliveringMissionClick = (): void => {
        onClickMqttPublish(mqttClient!, requestTaskTopic, deliveringMissionJSON);
    }

    const onReturningMissionClick = (): void => {
        onClickMqttPublish(mqttClient!, requestTaskTopic, retunringMissionJSON);
    }

    const onControlMoveToDestClick = (): void => {
        onClickMqttPublish(mqttClient!, requestTaskTopic, controlMoveToDestJSON);
    }

    const onControlMsCompleteReturnClick = (): void => {
        onClickMqttPublish(mqttClient!, requestTaskTopic, controlMsCompleteReturnJSON);
    }

    const onControlMsCompleteNoReturnClick = (): void => {
        onClickMqttPublish(mqttClient!, requestTaskTopic, controlMsCompleteNoReturnJSON);
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
                    mqttClient={mqttClient}
                    center={blueSpaceCoord}
                    state={mapState}
                />
            </div>
            <div className="request_component_container">
                <BlueSpaceRequestComponent
                    onStraightClick={onStraightClick}
                    onStraightHorizonTestClick={onStraightHorizonTestClick}
                    onCallClick={onCallClick}
                    onDeliveryClick={onDeliveryClick}
                    onWaitingClick={onWaitingClick}
                    onDeliveringMissionClick={onDeliveringMissionClick}
                    onReturningMissionClick={onReturningMissionClick}
                    onControlMoveToDestClick={onControlMoveToDestClick}
                    onControlMsCompleteReturnClick={onControlMsCompleteReturnClick}
                    onControlMsCompleteNoReturnClick={onControlMsCompleteNoReturnClick}
                />
            </div>
        </div>
    );
}

export default BlueSpaceDashBoardPage;