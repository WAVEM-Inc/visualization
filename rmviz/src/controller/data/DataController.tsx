import axios, { AxiosResponse } from "axios";
import mqtt, { IPublishPacket } from "mqtt/*";
import React, { useEffect, useReducer, useState } from "react";
import { Route, Switch } from "react-router-dom";
import Rqtt from "../../api/application/rqtt";
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
    const [rqtt, setRqtt] = useState<Rqtt>();
    const [rqttC, setRqttC] = useState<mqtt.MqttClient>();

    const requestTopicFormat: string = "net/wavem/rms/rqtt/mtr";
    const requestHeartBeatTopic: string = `${requestTopicFormat}/heartbeat`;

    const responseTopicFormat: string = "net/wavem/rms/rqtt/rtm";
    const responseCmdVelTopic: string = `${responseTopicFormat}/cmd_vel`;
    const responsePathFileSelectTopic: string = `${responseTopicFormat}/path/select`;

    const requiredResponseTopicList: Array<string> = [
        `${responseTopicFormat}/status/rbt_status`,
        `${responseTopicFormat}/status/service_status`,
        `${responseTopicFormat}/report/error_report`,
        `${responseTopicFormat}/report/control_report`,
        `${responseTopicFormat}/graph_list`,
        `${responseTopicFormat}/obstacle_detect`,
        `${responseTopicFormat}/lidar_signal`
    ];

    const httpLoadMQTTConfigRequest: Function = async (): Promise<void> => {
        try {
            const response: AxiosResponse<any, any> = await axios.post("/v1/api/mqtt/load/config");
            console.info(`MQTT : ${JSON.stringify(response)}`);
            setMqttData(response.data);
        } catch (error) {
            console.error("Error fetching MQTT data", error);
        }
    }

    useEffect((): void => {
        httpLoadMQTTConfigRequest();
    }, []);

    useEffect(() => {
        if (mqttData) {
            console.info(`mqttData : ${JSON.stringify(mqttData)}`);
            const rqtt: Rqtt = new Rqtt();
            setRqtt(rqtt);
            const rqttC: mqtt.MqttClient = rqtt.initialize(mqttData);
            setRqttC(rqttC);
        }
    }, [mqttData]);

    useEffect(() => {
        if (rqtt) {
            if (rqttC) {
                setInterval(() => {
                    const heartBeatJSON: any = {
                        "request_time": getCurrentTime()
                    };
                    rqtt.publish(rqttC, requestHeartBeatTopic, JSON.stringify(heartBeatJSON));
                }, 900);

                setTimeout(() => {
                    const isMqttConnected: boolean = rqttC.connected;
                    if (!isMqttConnected) {
                        alert("MQTT 연결을 확인하세요.");
                    }
                }, 5000);
            }
        }
    }, [rqtt, rqttC]);

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
                    rqtt={rqtt!}
                    rqttC={rqttC!}
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