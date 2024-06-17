import React, { useEffect } from "react";
import ROSComponent from "../../../components/ros/ROSComponent";
import TopComponent from "../../../components/top/TopComponent";
import "./KECROSPage.css";
import Rqtt from "../../../api/application/rqtt";
import mqtt from "mqtt/*";

interface KECROSPageProps {
    rqtt: Rqtt;
    rqttC: mqtt.MqttClient;
}

const KECROSPage: React.FC<KECROSPageProps> = ({
    rqtt,
    rqttC
}: KECROSPageProps): React.ReactElement<any, any> | null => {

    useEffect(() => {
        if (rqtt) {
            
        }
    }, []);

    return (
        <div className="path_search_container">
            <div className="top_component_container">
                <TopComponent
                    rqtt={rqtt}
                    rqttC={rqttC}
                 />
            </div>
            <div className="ros_component_container">
                <ROSComponent
                    rqtt={rqtt}
                    rqttC={rqttC}
                />
            </div>
        </div>
    )
}

export default KECROSPage;