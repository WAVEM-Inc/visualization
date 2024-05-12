import { IPublishPacket } from "mqtt/*";
import React, { useEffect, useReducer, useState } from "react";
import { Route, Switch } from "react-router-dom";
import MqttClient from "../../api/mqttClient";
import { SET_GPS, SET_GPS_FILTERED, SET_ODOM_EULAR, SET_PATH, SET_ROUTE_STATUS, initialMapState, mapStateReducer } from "../../domain/map/MapDomain";
import { SET_BATTERY, SET_HEARTBEAT, initialTopState, topStateReducer } from "../../domain/top/TopDomain";
import BlueSpaceDashBoardPage from "../../page/bluespace/dashBoard/BlueSpaceDashBoardPage";
import BlueSpaceDataBoardPage from "../../page/bluespace/databoard/BlueSpaceDataBoardPage";
import BlueSpaceROSPage from "../../page/bluespace/ros/BlueSpaceROSPage";
import { getCurrentTime } from "../../utils/Utils";
import { SET_URDF, initalROSState, rosStateReducer } from "../../domain/ros/ROSDomain";

const DataController: React.FC = (): React.ReactElement<any, any> | null => {
    const [topState, topStateDispatch] = useReducer(topStateReducer, initialTopState);
    const [mapState, mapStateDistpatch] = useReducer(mapStateReducer, initialMapState);
    const [rosState, rosStateDistpatch] = useReducer(rosStateReducer, initalROSState);
    const [responseData, setResponseData] = useState<any>({});

    const [mqttClient, setMqttClient] = useState<MqttClient>();

    const requestTopicFormat: string = "/rms/ktp/dummy/request";
    const requestHeartBeatTopic: string = `${requestTopicFormat}/heartbeat`;

    const responseTopicFormat: string = "/rms/ktp/dummy/response";
    const responsePathTopic: string = `${responseTopicFormat}/path`;
    const resposneGpsTopic: string = `${responseTopicFormat}/gps`;
    const resposneGpsFilteredTopic: string = `${responseTopicFormat}/gps/filtered`;
    const responseRouteStatusTopic: string = `${responseTopicFormat}/route/status`;
    const responseOdomEularTopic: string = `${responseTopicFormat}/odom/eular`;
    const responseHeartBeatTopic: string = `${responseTopicFormat}/heartbeat`;
    const responseBatteryTopic: string = `${responseTopicFormat}/battery/state`;
    const responseURDFTopic: string = `${responseTopicFormat}/urdf`;

    const requiredResponseTopicList: Array<string> = [
        `${responseTopicFormat}/rbt_status`,
        `${responseTopicFormat}/service_status`,
        `${responseTopicFormat}/error_report`,
        `${responseTopicFormat}/control_report`,
        `${responseTopicFormat}/graph_list`,
        `${responseTopicFormat}/obstacle_detect`,
        `${responseTopicFormat}/lidar_signal`
    ];

    const handleResponseMQTTCallback = (mqttClient: MqttClient): void => {
        mqttClient.client.on("message", (topic: string, payload: Buffer, packet: IPublishPacket) => {
            const message: any = JSON.parse(payload.toString());
            if (topic === responsePathTopic) {
                mapStateDistpatch({ type: SET_PATH, payload: message });
            } else if (topic === resposneGpsTopic) {
                mapStateDistpatch({ type: SET_GPS, payload: message });
            } else if (topic === resposneGpsFilteredTopic) {
                mapStateDistpatch({ type: SET_GPS_FILTERED, payload: message });
            } else if (topic === responseRouteStatusTopic) {
                mapStateDistpatch({ type: SET_ROUTE_STATUS, payload: message });
            } else if (topic === responseOdomEularTopic) {
                mapStateDistpatch({ type: SET_ODOM_EULAR, payload: message });
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

    const setUpResponseMQTTConnections = (mqttClient: MqttClient): void => {
        console.info(`${requiredResponseTopicList}`);
        for (const requiredTopic of requiredResponseTopicList) {
            mqttClient.subscribe(requiredTopic);
        }
    }

    useEffect(() => {
        const _mqttClient: MqttClient = new MqttClient();
        _mqttClient.subscribe(responsePathTopic);
        _mqttClient.subscribe(resposneGpsTopic);
        _mqttClient.subscribe(resposneGpsFilteredTopic);
        _mqttClient.subscribe(responseRouteStatusTopic);
        _mqttClient.subscribe(responseOdomEularTopic);
        _mqttClient.subscribe(responseHeartBeatTopic);
        _mqttClient.subscribe(responseBatteryTopic);
        _mqttClient.subscribe(responseURDFTopic);
        handleResponseMQTTCallback(_mqttClient);
        setUpResponseMQTTConnections(_mqttClient);
        setMqttClient(_mqttClient);

        setInterval(() => {
            const heartBeatJSON: any = {
                "request_time": getCurrentTime()
            };
            _mqttClient!.publish(requestHeartBeatTopic, JSON.stringify(heartBeatJSON));
        }, 1300);
    }, []);

    return (
        <Switch>
            <Route exact path={"/bluespace/ros"}>
                <BlueSpaceROSPage
                    mqttClient={mqttClient!}
                    topState={topState}
                    rosState={rosState}
                />
            </Route>
            <Route exact path={"/bluespace/dashboard"}>
                <BlueSpaceDashBoardPage
                    mqttClient={mqttClient!}
                    topState={topState}
                    mapState={mapState}
                />
            </Route>
            <Route exact path={"/bluespace/data"}>
                <BlueSpaceDataBoardPage
                    topState={topState}
                    responseData={responseData}
                />
            </Route>
        </Switch>
    );
}

export default DataController;