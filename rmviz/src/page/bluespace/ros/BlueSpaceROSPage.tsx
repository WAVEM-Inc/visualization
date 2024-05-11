import React from "react";
import MqttClient from "../../../api/mqttClient";
import TopComponent from "../../../components/top/TopComponent";
import { MapState } from "../../../domain/map/MapDomain";
import { TopState } from "../../../domain/top/TopDomain";
import "./BlueSpaceROSPage.css";

interface BlueSpaceROSPageProps {
    mqttClient: MqttClient;
    topState: TopState;
}

const BlueSpaceROSPage: React.FC<BlueSpaceROSPageProps> = ({
    mqttClient,
    topState
}: BlueSpaceROSPageProps): React.ReactElement<any, any> | null => {
    const blueSpaceCoord: naver.maps.LatLng = new naver.maps.LatLng(37.305985, 127.2401652);

    return (
        <div className="path_search_container">
            <div className="top_component_container">
                <TopComponent
                    state={topState}
                 />
            </div>
        </div>
    )
}

export default BlueSpaceROSPage;