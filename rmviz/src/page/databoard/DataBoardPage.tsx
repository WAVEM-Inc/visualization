import { IPublishPacket } from "mqtt/*";
import { useEffect, useState } from "react";
import MqttClient from "../../api/mqttClient";
import ResponseComponent from "../../components/response/ResponseComponent";
import './DataBoardPage.css';

export default function DataBoardPage() {
    const [mqttClient, setMqttClient] = useState<MqttClient>();
    const [responseData, setResponseData] = useState<any>({});
    const [lidarSwitch, setLiDARSwitch] = useState<boolean>(false);

    const responseTopicFormat: string = "/rms/ktp/dummy/response";

    const requiredResponseTopicList: Array<string> = [
        `${responseTopicFormat}/rbt_status`,
        `${responseTopicFormat}/service_status`,
        `${responseTopicFormat}/error_report`,
        `${responseTopicFormat}/control_report`,
        `${responseTopicFormat}/graph_list`,
        `${responseTopicFormat}/obstacle_detect`,
        `${responseTopicFormat}/lidar_signal`
    ];

    const filterJSON = (json: any): any => {
        const { default: removedDefault, ...newData } = json;

        return newData;
    }

    const setUpResponseMQTTConnections = (mqttClient: MqttClient): void => {
        console.info(`${requiredResponseTopicList}`);
        for (const requiredTopic of requiredResponseTopicList) {
            mqttClient.subscribe(requiredTopic);
        }
    }

    const handleResponseMQTTCallback = (mqttClient: MqttClient): void => {
        mqttClient.client.on("message", (topic: string, payload: Buffer, packet: IPublishPacket) => {
            console.info(`Response CB\n\ttopic : ${topic}\n\tpayload : ${JSON.stringify(JSON.parse(payload.toString()))}`);
            if (topic === requiredResponseTopicList[0]) {
                const newData: any = JSON.parse(payload.toString());
                setResponseData((prevData: any) => ({
                    ...prevData,
                    rbt_status: newData
                }));
            } else if (topic === requiredResponseTopicList[1]) {
                const newData: any = JSON.parse(payload.toString());
                setResponseData((prevData: any) => ({
                    ...prevData,
                    service_status: newData
                }));
            } else if (topic === requiredResponseTopicList[2]) {
                const newData: any = JSON.parse(payload.toString());
                setResponseData((prevData: any) => ({
                    ...prevData,
                    error_report: newData
                }));
            } else if (topic === requiredResponseTopicList[3]) {
                const newData: any = JSON.parse(payload.toString());
                setResponseData((prevData: any) => ({
                    ...prevData,
                    control_report: newData
                }));
            } else if (topic === requiredResponseTopicList[4]) {
                const newData: any = JSON.parse(payload.toString());
                setResponseData((prevData: any) => ({
                    ...prevData,
                    graph_list: newData
                }));
            } else if (topic === requiredResponseTopicList[5]) {
                const newData: any = JSON.parse(payload.toString());
                setResponseData((prevData: any) => ({
                    ...prevData,
                    obstacle_detect: newData
                }));
            } else if (topic === requiredResponseTopicList[6]) {
                const newData: any = JSON.parse(payload.toString());
                setResponseData((prevData: any) => ({
                    ...prevData,
                    lidar_signal: newData
                }));

                if (newData.signal_type === "START") {
                    setLiDARSwitch(true);
                } else {
                    setLiDARSwitch(false);
                }
            } else return;
        });
    }

    useEffect(() => {
        const mqttClient: MqttClient = new MqttClient();
        setMqttClient(mqttClient);
        
        setUpResponseMQTTConnections(mqttClient);
        handleResponseMQTTCallback(mqttClient);
    }, []);

    // setInterval(() => {
    //     if (lidarSwitch) {
    //         mqttClient!.publish("/rms/ktp/dummy/request/detected_object", JSON.stringify(filterJSON(dectectedObjectJSON)));
    //     } else {
    //         return;
    //     }
    // }, 500);

    return (
        <div className="data_board_container">
            <div className="top_component_container">
                {/* <TopComponents /> */}
            </div>
            <div className="response_component_container">
                <ResponseComponent
                    responseData={responseData} />
            </div>
        </div>
    );
};