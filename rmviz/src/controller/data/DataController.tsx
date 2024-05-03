import { IPublishPacket } from "mqtt/*";
import React, { useEffect, useReducer, useState } from "react";
import { Route, Switch } from "react-router-dom";
import MqttClient from "../../api/mqttClient";
import { SET_GPS, SET_GPS_FILTERED, SET_ODOM_EULAR, SET_PATH, SET_ROUTE_STATUS, initialMapState, mapStateReducer } from "../../domain/map/MapDomain";
import { SET_BATTERY, SET_HEARTBEAT, initialTopState, topStateReducer } from "../../domain/top/TopDomain";
import BlueSpaceDashBoardPage from "../../page/bluespace/BlueSpaceDashBoardPage";
import { getCurrentTime } from "../../utils/Utils";

const DataController: React.FC = (): React.ReactElement<any, any> | null => {
    const [topState, topStateDispatch] = useReducer(topStateReducer, initialTopState);
    const [mapState, mapStateDistpatch] = useReducer(mapStateReducer, initialMapState);

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

    const handleResponseMQTTCallback = (mqttClient: MqttClient): void => {
        mqttClient.client.on("message", (topic: string, payload: Buffer, packet: IPublishPacket) => {
            const message: any = JSON.parse(payload.toString())
            if (topic === responsePathTopic) {
                mapStateDistpatch({type: SET_PATH, payload: message});
            } else if (topic === resposneGpsTopic) {
                mapStateDistpatch({type: SET_GPS, payload: message});
            } else if (topic === resposneGpsFilteredTopic) {
                mapStateDistpatch({type: SET_GPS_FILTERED, payload: message});
            } else if (topic === responseRouteStatusTopic) {
                mapStateDistpatch({type: SET_ROUTE_STATUS, payload: message});
            } else if (topic === responseOdomEularTopic) {
                mapStateDistpatch({type: SET_ODOM_EULAR, payload: message});
            } else if (topic === responseHeartBeatTopic) {
                topStateDispatch({ type: SET_HEARTBEAT, payload: message });
            } else if (topic === responseBatteryTopic) {
                topStateDispatch({ type: SET_BATTERY, payload: message });
            } else {
                return;
            }
        });
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
        handleResponseMQTTCallback(_mqttClient);
        setMqttClient(_mqttClient);

        setInterval(() => {
            const heartBeatJSON: any = {
                "request_time": getCurrentTime()
            };
            _mqttClient!.publish(requestHeartBeatTopic, JSON.stringify(heartBeatJSON));
        }, 1000);
    }, []);
    
    return (
        <Switch>
            <Route exact path={"/bluespace"}>
                <BlueSpaceDashBoardPage
                    mqttClient={mqttClient!}
                    topState={topState}
                    mapState={mapState}
                />
            </Route>
        </Switch>
    );
}

export default DataController;