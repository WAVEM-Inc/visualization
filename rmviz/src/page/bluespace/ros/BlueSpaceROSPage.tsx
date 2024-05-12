import React, { useEffect } from "react";
import MqttClient from "../../../api/mqttClient";
import TopComponent from "../../../components/top/TopComponent";
import { TopState } from "../../../domain/top/TopDomain";
import { ROSState } from "../../../domain/ros/ROSDomain";
import ROSComponent from "../../../components/ros/ROSComponent";
import "./BlueSpaceROSPage.css";

interface BlueSpaceROSPageProps {
    mqttClient: MqttClient;
    topState: TopState;
    rosState: ROSState;
}

const BlueSpaceROSPage: React.FC<BlueSpaceROSPageProps> = ({
    mqttClient,
    topState,
    rosState
}: BlueSpaceROSPageProps): React.ReactElement<any, any> | null => {

    useEffect(() => {
        if (mqttClient) {
            const mqttURDFRequestTopic: string = "/rms/ktp/dummy/request/urdf";
            mqttClient.publish(mqttURDFRequestTopic, JSON.stringify({data: ""}));
        }
    }, []);

    return (
        <div className="path_search_container">
            <div className="top_component_container">
                <TopComponent
                    state={topState}
                 />
            </div>
            <div className="ros_component_container">
                <ROSComponent
                    state={rosState}
                />
            </div>
        </div>
    )
}

export default BlueSpaceROSPage;