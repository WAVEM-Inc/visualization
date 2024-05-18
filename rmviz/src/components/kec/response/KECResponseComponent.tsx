import React, { useEffect, useState } from "react";
import './KECResponseComponent.css';


interface KECResponseComponentProps {
    responseData: any;
}

const KECResponseComponent: React.FC<KECResponseComponentProps> = ({ responseData }: KECResponseComponentProps) => {
    const [rbtStatus, setRbtStatus] = useState<string>("");
    const [serviceStatus, setServiceStatus] = useState<string>("");
    const [errorReport, setErrorReport] = useState<string>("");
    const [controlReport, setControlReport] = useState<string>("");
    const [graphList, setGraphList] = useState<string>("");
    const [obstacleDetect, setObstacleDetect] = useState<string>("");


    useEffect(() => {
        setRbtStatus(JSON.stringify(responseData.rbt_status, null, 4));
        setServiceStatus(JSON.stringify(responseData.service_status, null, 4));
        setErrorReport(JSON.stringify(responseData.error_report, null, 4));
        setControlReport(JSON.stringify(responseData.control_report, null, 4));
        setGraphList(JSON.stringify(responseData.graph_list, null, 2));
        setObstacleDetect(JSON.stringify(responseData.obstacle_detect, null, 4));
    }, [responseData]);

    return (
        <div className="response_callback_grid_container">
            <div className={`grid_item rbt_status`}>
                <div className={"grid_item_title"}>rbt_status</div>
                <div className="grid_item_data_container">
                    <div className="grid_item_data">
                        <pre>{rbtStatus}</pre>
                    </div>
                </div>
            </div>
            <div className={`grid_item service_status`}>
                <div className={"grid_item_title"}>service_status</div>
                <div className="grid_item_data_container">
                    <div className="grid_item_data">
                        <pre>{serviceStatus}</pre>
                    </div>
                </div>
            </div>
            <div className={`grid_item error_report`}>
                <div className={"grid_item_title"}>error_report</div>
                <div className="grid_item_data_container">
                    <div className="grid_item_data">
                        <pre>{errorReport}</pre>
                    </div>
                </div>
            </div>
            <div className={`grid_item control_report`}>
                <div className={"grid_item_title"}>control_report</div>
                <div className="grid_item_data_container">
                    <div className="grid_item_data">
                        <pre>{controlReport}</pre>
                    </div>
                </div>
            </div>
            <div className={`grid_item graph_list`}>
                <div className={"grid_item_title"}>graph_list</div>
                <div className="grid_item_data_container">
                    <div className="grid_item_data">
                        <pre>{graphList}</pre>
                    </div>
                </div>
            </div>
            <div className={`grid_item obstacle_detect`}>
                <div className={"grid_item_title"}>obstacle_detect</div>
                <div className="grid_item_data_container">
                    <div className="grid_item_data">
                        <pre>{obstacleDetect}</pre>
                    </div>
                </div>
            </div>
        </div>
    );
};

export default KECResponseComponent;
