import { Wrapper } from "@googlemaps/react-wrapper";
import React, { useEffect, useState } from "react";
import MapComponent from "../../../components/map/MapComponent";
import TopComponent from "../../../components/top/TopComponent";
import "./KECDashBoardPage.css";
import mqtt from "mqtt/*";
import Rqtt from "../../../api/application/rqtt";

interface KECDashBoardPageProps {
    rqtt: Rqtt;
    rqttC: mqtt.MqttClient;
}

const KECDashBoardPage: React.FC<KECDashBoardPageProps> = ({
    rqtt,
    rqttC
}: KECDashBoardPageProps): React.ReactElement<any, any> | null => {

    return (
        <div className="dash_board_container">
            <div className="top_component_container">
                <TopComponent
                    rqtt={rqtt}
                    rqttC={rqttC}
                />
            </div>
            <div className="map_component_container">
                <Wrapper apiKey={`${process.env.GOOGLE_MAP_API_KEY}`}>
                    <MapComponent
                        rqtt={rqtt}
                        rqttC={rqttC}
                    />
                </Wrapper>
            </div>
        </div>
    );
}

export default KECDashBoardPage;