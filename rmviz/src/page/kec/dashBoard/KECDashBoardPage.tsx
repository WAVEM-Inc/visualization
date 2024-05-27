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
        </div>
    );
}

export default KECDashBoardPage;