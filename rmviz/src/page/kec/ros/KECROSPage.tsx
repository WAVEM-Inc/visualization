import React, { useEffect } from "react";
import MqttClient from "../../../api/mqttClient";
import ROSComponent from "../../../components/ros/ROSComponent";
import TopComponent from "../../../components/top/TopComponent";
import { ROSState } from "../../../domain/ros/ROSDomain";
import { TopState } from "../../../domain/top/TopDomain";
import "./KECROSPage.css";

interface KECROSPageProps {
    mqttClient: MqttClient;
    topState: TopState;
    rosState: ROSState;
}

const KECROSPage: React.FC<KECROSPageProps> = ({
    mqttClient,
    topState,
    rosState
}: KECROSPageProps): React.ReactElement<any, any> | null => {

    useEffect(() => {
        if (mqttClient) {
            const mqttURDFRequestTopic: string = "/rms/ktp/dummy/request/urdf";
            mqttClient.publish(mqttURDFRequestTopic, JSON.stringify({data: ""}));
        }
    }, []);

    return (
        <div className="path_search_container">
            {/* <div className="top_component_container">
                <TopComponent
                    state={topState}
                 />
            </div> */}
            <div className="ros_component_container">
                <ROSComponent
                    state={rosState}
                />
            </div>
        </div>
    )
}

export default KECROSPage;