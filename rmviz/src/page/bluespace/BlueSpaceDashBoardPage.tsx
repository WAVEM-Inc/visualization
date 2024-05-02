import { IPublishPacket } from "mqtt/*";
import { useEffect, useState } from "react";
import MqttClient from "../../api/mqttClient";
import * as callJSON from "../../assets/json/bluespace/secondary/call.json";
import * as deliveryJSON from "../../assets/json/bluespace/secondary/delivery.json";
import * as straightJSON from "../../assets/json/bluespace/secondary/straight.json";
import * as straightTestJSON from "../../assets/json/bluespace/secondary/straight_test.json";
import * as waitingJSON from "../../assets/json/bluespace/secondary/waiting.json";
import * as emergencyResumeJSON from "../../assets/json/common/emergency_resume.json";
import * as emergencyStopJSON from "../../assets/json/common/emergency_stop.json";
import BlueSpaceRequestComponent from "../../components/bluespace/BlueSpaceRequestComponent";
import MapComponent from "../../components/map/MapComponents";
import TopComponents from "../../components/top/TopComponent";
import { getCurrentTime, onClickMqttPublish } from "../../utils/Utils";
import "./BlueSpaceDashBoardPage.css";

export default function BlueSpaceDashBoardPage() {

    const [mqttClient, setMqttClient] = useState<MqttClient | undefined>();
    const [pathData, setPathData] = useState<any>(null);
    const [gpsData, setGpsData] = useState<any>(null);
    const [gpsFilteredData, setGpsFilteredData] = useState<any>(null);
    const [routeStatus, setRouteStatus] = useState<any>(null);
    const [odomEularData, setOdomEularData] = useState<any>(null);
    const [heartBeat, setHeartBeat] = useState<any>(null);
    const [battery, setBattery] = useState<number>(0.0);

    const blueSpaceCoord: naver.maps.LatLng = new naver.maps.LatLng(37.305985, 127.2401652);
    const requestTopicFormat: string = "/rms/ktp/dummy/request";
    const requestRouteToPoseTopic: string = `${requestTopicFormat}/route_to_pose`;
    const requestEmergencyTopic: string = `${requestTopicFormat}/can/emergency`;
    const requestGoalCancelTopic: string = `${requestTopicFormat}/goal/cancel`;
    const requestInitTopic: string = `${requestTopicFormat}/can/init`;
    const requestHeartBeatTopic: string = `${requestTopicFormat}/heartbeat`;

    const responseTopicFormat: string = "/rms/ktp/dummy/response";
    const responsePathTopic: string = `${responseTopicFormat}/path`;
    const resposneGpsTopic: string = `${responseTopicFormat}/gps`;
    const resposneGpsFilteredTopic: string = `${responseTopicFormat}/gps/filtered`;
    const responseRouteStatusTopic: string = `${responseTopicFormat}/route/status`;
    const responseOdomEularTopic: string = `${responseTopicFormat}/odom/eular`;
    const responseHeartBeatTopic: string = `${responseTopicFormat}/heartbeat`;
    const responseBatteryTopic: string = `${responseTopicFormat}/battery/state`;

    const onStraightClick = (): void => {
        onClickMqttPublish(mqttClient!, requestRouteToPoseTopic, straightJSON);
    }

    const onStraightTestClick = (): void => {
        onClickMqttPublish(mqttClient!, requestRouteToPoseTopic, straightTestJSON);
    }

    const onCallClick = (): void => {
        onClickMqttPublish(mqttClient!, requestRouteToPoseTopic, callJSON);
    }

    const onDeliveryClick = (): void => {
        onClickMqttPublish(mqttClient!, requestRouteToPoseTopic, deliveryJSON);
    }

    const onWaitingClick = (): void => {
        onClickMqttPublish(mqttClient!, requestRouteToPoseTopic, waitingJSON);
    }

    const onEmergencyStopClick = (): void => {
        onClickMqttPublish(mqttClient!, requestEmergencyTopic, emergencyStopJSON);
    }

    const onEmergencyResumeClick = (): void => {
        onClickMqttPublish(mqttClient!, requestEmergencyTopic, emergencyResumeJSON);
    }

    const onGoalCancelClick = (): void => {
        onClickMqttPublish(mqttClient!, requestGoalCancelTopic, {
            "cancel": true
        });
        setPathData(null);
    }

    const onInitClick = (): void => {
        onClickMqttPublish(mqttClient!, requestInitTopic, {
            "can_sign_tran_state": true
        });
        setPathData(null);
    }

    const handleResponseMQTTCallback = (mqttClient: MqttClient): void => {
        mqttClient.client.on("message", (topic: string, payload: Buffer, packet: IPublishPacket) => {
            const message: any = JSON.parse(payload.toString())
            if (topic === responsePathTopic) {
                console.info(`Path cb : ${JSON.stringify(message)}`);
                setPathData(message);
            } else if (topic === resposneGpsTopic) {
                setGpsData(message);
            } else if (topic === resposneGpsFilteredTopic) {
                setGpsFilteredData(message);
            } else if (topic === responseRouteStatusTopic) {
                setRouteStatus(message);
            } else if (topic === responseOdomEularTopic) {
                setOdomEularData(message);
            } else if (topic === responseHeartBeatTopic) {
                setHeartBeat(message);
            } else if (topic === responseBatteryTopic) {
                setBattery(message);
            } else {
                return;
            }
        });
    }

    useEffect(() => {
        const mqttClient: MqttClient = new MqttClient();
        setMqttClient(mqttClient);
        mqttClient.subscribe(responsePathTopic);
        mqttClient.subscribe(resposneGpsTopic);
        mqttClient.subscribe(resposneGpsFilteredTopic);
        mqttClient.subscribe(responseRouteStatusTopic);
        mqttClient.subscribe(responseOdomEularTopic);
        mqttClient.subscribe(responseHeartBeatTopic);
        mqttClient.subscribe(responseBatteryTopic);
        handleResponseMQTTCallback(mqttClient);

        setInterval(() => {
            const heartBeatJSON: any = {
                "request_time": getCurrentTime()
            };
            mqttClient!.publish(requestHeartBeatTopic, JSON.stringify(heartBeatJSON));
        }, 1500);
    }, []);

    useEffect(() => {
        if (routeStatus) {
            console.info(`routeStatus : ${JSON.stringify(routeStatus)}`);
        } else {
            return;
        }

        return () => {
            setRouteStatus(null);
        }
    }, [routeStatus]);

    return (
        <div className="dash_board_container">
            <div className="top_component_container">
                <TopComponents
                    heartBeatData={heartBeat}
                    batteryData={battery}
                 />
            </div>
            <div className="map_component_container">
                <MapComponent
                    center={blueSpaceCoord}
                    pathData={pathData}
                    gpsData={gpsData}
                    gpsFilteredData={gpsFilteredData}
                    odomEularData={odomEularData}
                    routeStatus={routeStatus}
                    onEmergencyStopClick={onEmergencyStopClick}
                    onEmergencyResumeClick={onEmergencyResumeClick}
                    onGoalCancelClick={onGoalCancelClick}
                    onInitClick={onInitClick}
                />
            </div>
            <div className="request_component_container">
                <BlueSpaceRequestComponent
                    onStraightClick={onStraightClick}
                    onStraightTestClick={onStraightTestClick}
                    onCallClick={onCallClick}
                    onDeliveryClick={onDeliveryClick}
                    onWaitingClick={onWaitingClick}
                />
            </div>
        </div>
    );
}