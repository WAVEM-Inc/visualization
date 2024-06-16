import mqtt from "mqtt/*";
import React from "react";
import Rqtt from "../../../api/application/rqtt";
import TopComponent from "../../../components/top/TopComponent";
import './KECDataBoardPage.css';

interface KECDataBoardPageProps {
    rqtt: Rqtt;
    rqttC: mqtt.MqttClient;
}

const KECDataBoardPage: React.FC<KECDataBoardPageProps> = ({
    rqtt,
    rqttC
}): React.ReactElement<any, any> | null => {

    return (
        <div className="data_board_container">
            <div className="top_component_container">
                <TopComponent 
                    rqtt={rqtt}
                    rqttC={rqttC}
                />
            </div>
            <div className="response_component_container">
                {/* <KECResponseComponent responseData={responseData} /> */}
            </div>
        </div>
    );
};

export default KECDataBoardPage