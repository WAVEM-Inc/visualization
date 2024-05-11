import { IPublishPacket } from "mqtt/*";
import React, { useEffect, useState } from "react";
import MqttClient from "../../../api/mqttClient";
import ResponseComponent from "../../../components/response/ResponseComponent";
import TopComponent from "../../../components/top/TopComponent";
import { TopState } from "../../../domain/top/TopDomain";
import './BlueSpaceDataBoardPage.css';

interface BlueSpaceDataBoardPageProps {
    topState: TopState;
    responseData: any;
}

const BlueSpaceDataBoardPage: React.FC<BlueSpaceDataBoardPageProps> = ({
    topState,
    responseData
}): React.ReactElement<any, any> | null => {

    return (
        <div className="data_board_container">
            <div className="top_component_container">
                <TopComponent 
                    state={topState}
                />
            </div>
            <div className="response_component_container">
                <ResponseComponent
                    responseData={responseData} />
            </div>
        </div>
    );
};

export default BlueSpaceDataBoardPage