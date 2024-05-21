import React from "react";
import "./KECPathEditPage.css";
import { TopState } from "../../../domain/top/TopDomain";
import MqttClient from "../../../api/mqttClient";
import TopComponent from "../../../components/top/TopComponent";
import GoogleMapComponent from "../../../components/map/GoogleMapComponent";
import { MapState } from "../../../domain/map/MapDomain";
import { Wrapper } from "@googlemaps/react-wrapper";

interface KECPathEditPageProps {
    mqttClient: MqttClient;
    topState: TopState;
    mapState: MapState;
}

const KECPathEditPage: React.FC<KECPathEditPageProps> = ({
    mqttClient,
    topState,
    mapState
}: KECPathEditPageProps): React.ReactElement<any, any> => {
    return (
        <div>
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

export default KECPathEditPage;