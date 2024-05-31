import { Wrapper } from "@googlemaps/react-wrapper";
import React, { useEffect, useState } from "react";
import MqttClient from "../../../api/mqttClient";
import MapComponent from "../../../components/map/MapComponent";
import TopComponent from "../../../components/top/TopComponent";
import { MapState } from "../../../domain/map/MapDomain";
import { TopState } from "../../../domain/top/TopDomain";
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
                    <MapComponent
                        mqttClient={mqttClient!}
                        state={mapState}
                    />
                </Wrapper>
            </div>
        </div>
    );
}

export default KECDashBoardPage;