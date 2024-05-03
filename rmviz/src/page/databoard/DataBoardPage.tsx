import { IPublishPacket } from "mqtt/*";
import React, { useEffect, useState } from "react";
import MqttClient from "../../api/mqttClient";
import ResponseComponent from "../../components/response/ResponseComponent";
import TopComponent from "../../components/top/TopComponent";
import { TopState } from "../../domain/top/TopDomain";
import './DataBoardPage.css';

interface DataBoardPageProps {
    topState: TopState;
    responseData: any;
}

const DataBoardPage: React.FC<DataBoardPageProps> = ({
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

export default DataBoardPage