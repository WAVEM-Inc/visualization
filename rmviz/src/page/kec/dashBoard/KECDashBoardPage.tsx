import { Wrapper } from "@googlemaps/react-wrapper";
import React, { useEffect, useState } from "react";
import MqttClient from "../../../api/mqttClient";
import * as call01JSON from "../../../assets/json/kec/gumi/call_01.json";
import * as call04JSON from "../../../assets/json/kec/gumi/call_04.json";
import * as delivery02JSON from "../../../assets/json/kec/gumi/delivery_02.json";
import * as delivery05JSON from "../../../assets/json/kec/gumi/delivery_05.json";
import * as returning03JSON from "../../../assets/json/kec/gumi/returning_03.json";
import * as returning06JSON from "../../../assets/json/kec/gumi/returning_06.json";
import * as temp01JSON from "../../../assets/json/kec/gumi/temp_01.json";
import * as temp02JSON from "../../../assets/json/kec/gumi/temp_02.json";
import * as temp03JSON from "../../../assets/json/kec/gumi/temp_03.json";
import * as controlMoveToDestJSON from "../../../assets/json/control_movetodest.json";
import * as controlMsCompleteNoReturnJSON from "../../../assets/json/control_mscomplete_no_return.json";
import * as controlMsCompleteReturnJSON from "../../../assets/json/control_mscomplete_return.json";
import * as controlGrapySyncJSON from "../../../assets/json/control_graphsync.json";
import * as deliveringMissionJSON from "../../../assets/json/mission_delivering.json";
import * as retunringMissionJSON from "../../../assets/json/mission_returning.json";
import KECRequestComponent from "../../../components/kec/request/KECRequestComponent";
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
    const requestTopicFormat: string = "/rmviz/request";
    const requestRouteToPoseTopic: string = `${requestTopicFormat}/route_to_pose`;
    const requestTaskTopic: string = `${requestTopicFormat}/task`;

    const buildPathJSON: Function = (path: any): any => {
        const pathJSON: any = {
            isEnableToCommandRoute: isEnableToCommandRoute,
            path: path
        }

        return pathJSON;
    }

    const onCall01Click = (): void => {
        onClickMqttPublish(mqttClient!, requestRouteToPoseTopic, buildPathJSON(call01JSON));
    }

    const onDelivery02NCall02Click = (): void => {
        onClickMqttPublish(mqttClient!, requestRouteToPoseTopic, buildPathJSON(delivery02JSON));
    }

    const onWaiting03Click = (): void => {
        onClickMqttPublish(mqttClient!, requestRouteToPoseTopic, buildPathJSON(returning03JSON));
    }

    const onCall04Click = (): void => {
        onClickMqttPublish(mqttClient!, requestRouteToPoseTopic, buildPathJSON(call04JSON));
    }

    const onDelivery05NCall05Click = (): void => {
        onClickMqttPublish(mqttClient!, requestRouteToPoseTopic, buildPathJSON(delivery05JSON));
    }

    const onWaiting06Click = (): void => {
        onClickMqttPublish(mqttClient!, requestRouteToPoseTopic, buildPathJSON(returning06JSON));
    }

    const onTemp01Click = (): void => {
        onClickMqttPublish(mqttClient!, requestRouteToPoseTopic, buildPathJSON(temp01JSON));
    }

    const onTemp02Click = (): void => {
        onClickMqttPublish(mqttClient!, requestRouteToPoseTopic, buildPathJSON(temp02JSON));
    }

    const onTemp03Click = (): void => {
        onClickMqttPublish(mqttClient!, requestRouteToPoseTopic, buildPathJSON(temp03JSON));
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

    useEffect(() => {
        setIsEnableToCommandRoute(localStorage.getItem("isEnableToCommandRoute?"));
    }, [localStorage.getItem("isEnableToCommandRoute?")]);

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
                    onCall01Click={onCall01Click}
                    onDelivery02NCall02Click={onDelivery02NCall02Click}
                    onWaiting03Click={onWaiting03Click}
                    onCall04Click={onCall04Click}
                    onDelivery05NCall05Click={onDelivery05NCall05Click}
                    onWaiting06Click={onWaiting06Click}
                    onTemp01Click={onTemp01Click}
                    onTemp02Click={onTemp02Click}
                    onTemp03Click={onTemp03Click}
                    onDeliveringMissionClick={onDeliveringMissionClick}
                    onReturningMissionClick={onReturningMissionClick}
                    onControlMoveToDestClick={onControlMoveToDestClick}
                    onControlMsCompleteReturnClick={onControlMsCompleteReturnClick}
                    onControlMsCompleteNoReturnClick={onControlMsCompleteNoReturnClick}
                    onControlGraphSyncClick={onControlGraphSyncClick}
                />
            </div>
        </div>
    );
}

export default KECDashBoardPage;