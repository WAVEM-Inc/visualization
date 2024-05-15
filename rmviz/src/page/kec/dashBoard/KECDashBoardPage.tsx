import { Wrapper } from "@googlemaps/react-wrapper";
import React, { useEffect, useState } from "react";
import MqttClient from "../../../api/mqttClient";
import * as controlMoveToDestJSON from "../../../assets/json/control_movetodest.json";
import * as controlMsCompleteNoReturnJSON from "../../../assets/json/control_mscomplete_no_return.json";
import * as controlMsCompleteReturnJSON from "../../../assets/json/control_mscomplete_return.json";
import * as callJSON from "../../../assets/json/kec/secondary/call.json";
import * as deliveryJSON from "../../../assets/json/kec/secondary/delivery.json";
import * as straightJSON from "../../../assets/json/kec/secondary/straight.json";
import * as waitingJSON from "../../../assets/json/kec/secondary/waiting.json";
import * as deliveringMissionJSON from "../../../assets/json/mission_delivering.json";
import * as retunringMissionJSON from "../../../assets/json/mission_returning.json";
import KECRequestComponent from "../../../components/kec/request/RequestComponent";
import GoogleMapComponent from "../../../components/map/GoogleMapComponent";
import TopComponent from "../../../components/top/TopComponent";
import { MapState } from "../../../domain/map/MapDomain";
import { TopState } from "../../../domain/top/TopDomain";
import { onClickMqttPublish } from "../../../utils/Utils";
import "./KECDashBoardPage.css";

interface KECDashBoardPageProps {
    mqttClient: MqttClient;
    topState: TopState;
    mapState: MapState;
}

const KECDashBoardPage: React.FC<KECDashBoardPageProps> = ({
    mqttClient,
    topState,
    mapState
}: KECDashBoardPageProps): React.ReactElement<any, any> | null => {
    const [isEnableToCommandRoute, setIsEnableToCommandRoute] = useState<string | null>(null);
    const requestTopicFormat: string = "/rms/ktp/dummy/request";
    const requestRouteToPoseTopic: string = `${requestTopicFormat}/route_to_pose`;
    const requestTaskTopic: string = `${requestTopicFormat}/task`;

    useEffect(() => {
        setIsEnableToCommandRoute(localStorage.getItem("isEnableToCommandRoute?"));
    }, [localStorage.getItem("isEnableToCommandRoute?")]);

    const onStraightClick = (): void => {
        const j: any = {
            isEnableToCommandRoute: isEnableToCommandRoute,
            path: straightJSON
        };

        onClickMqttPublish(mqttClient!, requestRouteToPoseTopic, j);
    }

    const onCallClick = (): void => {
        const j: any = {
            isEnableToCommandRoute: isEnableToCommandRoute,
            path: callJSON
        };

        onClickMqttPublish(mqttClient!, requestRouteToPoseTopic, j);
    }

    const onDeliveryClick = (): void => {
        const j: any = {
            isEnableToCommandRoute: isEnableToCommandRoute,
            path: deliveryJSON
        };

        onClickMqttPublish(mqttClient!, requestRouteToPoseTopic, j);
    }

    const onWaitingClick = (): void => {
        const j: any = {
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

    return (
        <div className="dash_board_container">
            <div className="top_component_container">
                <TopComponent
                    state={topState}
                />
            </div>
            <div className="map_component_container">
                <Wrapper apiKey={`${process.env.GOOGLE_MAP_API_KEY}`}>
                    <GoogleMapComponent
                        mqttClient={mqttClient!}
                        state={mapState}
                    />
                </Wrapper>
            </div>
            <div className="request_component_container">
                <KECRequestComponent
                    onStraightClick={onStraightClick}
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

export default KECDashBoardPage;