import React from "react";
import KECResponseComponent from "../../../components/kec/response/ResponseComponent";
import TopComponent from "../../../components/top/TopComponent";
import { TopState } from "../../../domain/top/TopDomain";
import './KECDataBoardPage.css';

interface KECDataBoardPageProps {
    topState: TopState;
    responseData: any;
}

const KECDataBoardPage: React.FC<KECDataBoardPageProps> = ({
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
                <KECResponseComponent
                    responseData={responseData} />
            </div>
        </div>
    );
};

export default KECDataBoardPage