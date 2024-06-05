import axios, { AxiosResponse } from "axios";
import { IPublishPacket } from "mqtt/*";
import React, { useEffect, useReducer, useState } from "react";
import { Route, Switch } from "react-router-dom";
import MqttClient from "../../api/mqttClient";
import { SET_CMD_VEL, SET_GPS, SET_GPS_FILTERED, SET_ODOM_EULAR, SET_PATH, SET_ROUTE_STATUS, initialMapState, mapStateReducer } from "../../domain/map/MapDomain";
import { SET_URDF, initalROSState, rosStateReducer } from "../../domain/ros/ROSDomain";
import { SET_BATTERY, SET_HEARTBEAT, initialTopState, topStateReducer } from "../../domain/top/TopDomain";
import KECDashBoardPage from "../../page/kec/dashBoard/KECDashBoardPage";
import KECDataBoardPage from "../../page/kec/databoard/KECDataBoardPage";
import KECROSPage from "../../page/kec/ros/KECROSPage";
import { getCurrentTime } from "../../utils/Utils";

if (process.env.REACT_APP_API_BASE_URL) {
    axios.defaults.baseURL = process.env.REACT_APP_API_BASE_URL;
}

const DataController: React.FC = (): React.ReactElement<any, any> | null => {
    const [topState, topStateDispatch] = useReducer(topStateReducer, initialTopState);
    const [mapState, mapStateDistpatch] = useReducer(mapStateReducer, initialMapState);
    const [rosState, rosStateDistpatch] = useReducer(rosStateReducer, initalROSState);
    const [responseData, setResponseData] = useState<any>({});

    const [mqttData, setMqttData] = useState(null);
    const [mqttClient, setMqttClient] = useState<MqttClient>();

    const requestTopicFormat: string = "net/wavem/rms/rqtt/mtr";
    const requestHeartBeatTopic: string = `${requestTopicFormat}/heartbeat`;

    const responseTopicFormat: string = "net/wavem/rms/rqtt/rtm";
    const responsePathTopic: string = `${responseTopicFormat}/route/path`;
    const resposneGpsTopic: string = `${responseTopicFormat}/sensor/ublox/fix`;
    const resposneGpsFilteredTopic: string = `${responseTopicFormat}/gps/filtered`;
    const responseRouteStatusTopic: string = `${responseTopicFormat}/route/status`;
    const responseOdomEularTopic: string = `${responseTopicFormat}/drive/odom/eular`;
    const responseHeartBeatTopic: string = `${responseTopicFormat}/heartbeat`;
    const responseBatteryTopic: string = `${responseTopicFormat}/sensor/battery/state`;
    const responseURDFTopic: string = `${responseTopicFormat}/urdf`;
    const responseCmdVelTopic: string = `${responseTopicFormat}/cmd_vel`;
    const responsePathFileSelectTopic: string = `${responseTopicFormat}/path/select`;

    const requiredResponseTopicList: Array<string> = [
        `${responseTopicFormat}/rbt_status`,
        `${responseTopicFormat}/service_status`,
        `${responseTopicFormat}/error_report`,
        `${responseTopicFormat}/control_report`,
        `${responseTopicFormat}/graph_list`,
        `${responseTopicFormat}/obstacle_detect`,
        `${responseTopicFormat}/lidar_signal`
    ];

    const handleResponseMQTTCallback: Function = (mqttClient: MqttClient): void => {
        mqttClient.client.on("message", (topic: string, payload: Buffer, packet: IPublishPacket) => {
            const message: any = JSON.parse(payload.toString());
            if (topic === responsePathTopic || topic === responsePathFileSelectTopic) {
                mapStateDistpatch({ type: SET_PATH, payload: message });
            } else if (topic === resposneGpsTopic) {
                mapStateDistpatch({ type: SET_GPS, payload: message });
            } else if (topic === resposneGpsFilteredTopic) {
                mapStateDistpatch({ type: SET_GPS_FILTERED, payload: message });
            } else if (topic === responseRouteStatusTopic) {
                mapStateDistpatch({ type: SET_ROUTE_STATUS, payload: message });
            } else if (topic === responseOdomEularTopic) {
                mapStateDistpatch({ type: SET_ODOM_EULAR, payload: message });
            } else if (topic === responseCmdVelTopic) {
                mapStateDistpatch({ type: SET_CMD_VEL, payload: message });
            } else if (topic === responseHeartBeatTopic) {
                topStateDispatch({ type: SET_HEARTBEAT, payload: message });
            } else if (topic === responseBatteryTopic) {
                topStateDispatch({ type: SET_BATTERY, payload: message });
            } else if (topic === responseURDFTopic) {
                rosStateDistpatch({ type: SET_URDF, payload: payload.toString() });
            } else if (topic === requiredResponseTopicList[0]) {
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
            } else {
                return;
            }
        });
    }

    const setUpResponseMQTTConnections: Function = (mqttClient: MqttClient): void => {
        console.info(`${requiredResponseTopicList}`);
        for (const requiredTopic of requiredResponseTopicList) {
            mqttClient.subscribe(requiredTopic);
        }
    }

    const loadMQTTConfig: Function = async (): Promise<void> => {
        try {
            const response: AxiosResponse<any, any> = await axios.get("/v1/api/mqtt/load/config");
            setMqttData(response.data);
        } catch (error) {
            console.error("Error fetching MQTT data", error);
        }
    }

    useEffect((): void => {
        loadMQTTConfig();
    }, []);

    useEffect(() => {
        if (mqttData) {
            console.info(`mqttData : ${JSON.stringify(mqttData)}`);
            const _mqttClient: MqttClient = new MqttClient(mqttData);

            _mqttClient.subscribe(responsePathTopic);
            _mqttClient.subscribe(resposneGpsTopic);
            _mqttClient.subscribe(resposneGpsFilteredTopic);
            _mqttClient.subscribe(responseRouteStatusTopic);
            _mqttClient.subscribe(responseOdomEularTopic);
            _mqttClient.subscribe(responseHeartBeatTopic);
            _mqttClient.subscribe(responseBatteryTopic);
            _mqttClient.subscribe(responseURDFTopic);
            _mqttClient.subscribe(responseCmdVelTopic);
            _mqttClient.subscribe(responsePathFileSelectTopic);

            handleResponseMQTTCallback(_mqttClient);
            setUpResponseMQTTConnections(_mqttClient);
            setMqttClient(_mqttClient);
        }
    }, [mqttData]);

    useEffect(() => {
        if (mqttClient) {
            setInterval(() => {
                const heartBeatJSON: any = {
                    "request_time": getCurrentTime()
                };
                mqttClient!.publish(requestHeartBeatTopic, JSON.stringify(heartBeatJSON));
            }, 900);

            setTimeout(() => {
                const isMqttConnected: boolean = mqttClient.isConnected();
                if (!isMqttConnected) {
                    alert("MQTT 연결을 확인하세요.");
                }
            }, 5000);
        }
    }, [mqttClient]);

    return (
        <Switch>
            <Route exact path={"/kec/ros"}>
                <KECROSPage
                    mqttClient={mqttClient!}
                    topState={topState}
                    rosState={rosState}
                />
            </Route>
            <Route exact path={"/kec/dashboard"}>
                <KECDashBoardPage
                    mqttClient={mqttClient!}
                    topState={topState}
                    mapState={mapState}
                />
            </Route>
            <Route exact path={"/kec/data"}>
                <KECDataBoardPage
                    topState={topState}
                    responseData={responseData}
                />
            </Route>
        </Switch>
    );
}

export default DataController;