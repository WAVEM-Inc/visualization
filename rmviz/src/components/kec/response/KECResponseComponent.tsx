import mqtt from "mqtt/*";
import React, { useEffect, useState } from "react";
import Rqtt from "../../../api/application/rqtt";
import { RTM_TOPIC_FORMAT } from "../../../api/domain/common.constants";
import './KECResponseComponent.css';


interface KECResponseComponentProps {
    rqtt: Rqtt;
    rqttC: mqtt.MqttClient;
}

const KECResponseComponent: React.FC<KECResponseComponentProps> = ({
    rqtt,
    rqttC
}: KECResponseComponentProps) => {
    const [rbtStatus, setRbtStatus] = useState<string>("");
    const [serviceStatus, setServiceStatus] = useState<string>("");
    const [errorReport, setErrorReport] = useState<string>("");
    const [controlReport, setControlReport] = useState<string>("");
    const [graphList, setGraphList] = useState<string>("");
    const [obstacleDetect, setObstacleDetect] = useState<string>("");

    const rbtStatusCallback: (message: any) => void = (message: any): void => {
        setRbtStatus(JSON.stringify(message, null, 4));
    }

    const serviceStatuscallback: (message: any) => void = (message: any): void => {
        setServiceStatus(JSON.stringify(message, null, 4));
    }

    const errorReportCallback: (message: any) => void = (message: any): void => {
        setErrorReport(JSON.stringify(message, null, 4));
    }

    const controlReportCallback: (message: any) => void = (message: any): void => {
        setControlReport(JSON.stringify(message, null, 4));
    }

    const graphListCallbak: (message: any) => void = (message: any): void => {
        setGraphList(JSON.stringify(message, null, 4));
    }

    const obstacleDetectCallback: (message: any) => void = (message: any): void => {
        setObstacleDetect(JSON.stringify(message, null, 4));
    }

    useEffect(() => {
        if (rqtt) {
            if (rqttC) {
                const rbtStatusTopic: string = `${RTM_TOPIC_FORMAT}/status/rbt_status`;
                rqtt.subscribe(rqttC, rbtStatusTopic);
                rqtt.addSubscriptionCallback(rqttC, rbtStatusTopic, rbtStatusCallback);

                const serviceStatusTopic: string = `${RTM_TOPIC_FORMAT}/status/service_status`;
                rqtt.subscribe(rqttC, serviceStatusTopic);
                rqtt.addSubscriptionCallback(rqttC, serviceStatusTopic, serviceStatuscallback);

                const errorReportTopic: string = `${RTM_TOPIC_FORMAT}/report/error_report`;
                rqtt.subscribe(rqttC, errorReportTopic);
                rqtt.addSubscriptionCallback(rqttC, errorReportTopic, errorReportCallback);

                const controlReportTopic: string = `${RTM_TOPIC_FORMAT}/report/control_report`;
                rqtt.subscribe(rqttC, controlReportTopic);
                rqtt.addSubscriptionCallback(rqttC, controlReportTopic, controlReportCallback);

                const graphListTopic: string = `${RTM_TOPIC_FORMAT}/report/graph_list`;
                rqtt.subscribe(rqttC, graphListTopic);
                rqtt.addSubscriptionCallback(rqttC, graphListTopic, graphListCallbak);

                const obstacleDetectTopic: string = `${RTM_TOPIC_FORMAT}/obstacle_detect`;
                rqtt.subscribe(rqttC, obstacleDetectTopic);
                rqtt.addSubscriptionCallback(rqttC, obstacleDetectTopic, obstacleDetectCallback);
            }
        }
    }, [rqtt, rqttC]);

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
