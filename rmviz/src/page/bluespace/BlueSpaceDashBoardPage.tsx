import "./BlueSpaceDashBoardPage.css";
import { IPublishPacket } from "mqtt/*";
import { useEffect, useState } from "react";
import MqttClient from "../../api/mqttClient";
import MapComponent from "../../components/map/MapComponents";
import TopComponents from "../../components/top/TopComponent";
import BlueSpaceRequestComponent from "../../components/bluespace/BlueSpaceRequestComponent";
import { KTP_DEV_ID, filterJSON, getCurrentTime, onClickMqttPublish } from "../../utils/Utils";
import * as straightJSON from "../../assets/json/bluespace/straight.json";
import * as intersection1JSON from "../../assets/json/bluespace/intersection1.json";
import * as intersection2JSON from "../../assets/json/bluespace/intersection2.json";
import * as loadingJSON from "../../assets/json/bluespace/loading.json";
import * as landing1JSON from "../../assets/json/bluespace/landing1.json";
import * as landing2JSON from "../../assets/json/bluespace/landing2.json";
import * as gpsShadow1JSON from "../../assets/json/bluespace/gps_shadow1.json";
import * as gpsShadow2JSON from "../../assets/json/bluespace/gps_shadow2.json";
import * as emergencyStopJSON from "../../assets/json/common/emergency_stop.json";
import * as emergencyResumeJSON from "../../assets/json/common/emergency_resume.json";

export default function BlueSpaceDashBoardPage() {
    const [mqttClient, setMqttClient] = useState<MqttClient | undefined>();
    const [pathData, setPathData] = useState<any>(null);
    const [gpsData, setGpsData] = useState<any>(null);

    const requestTopicFormat: string = "/rms/ktp/dummy/request";
    const requestRouteToPoseTopic: string = `${requestTopicFormat}/route_to_pose`;
    const requestEmergencyTopic: string = `${requestRouteToPoseTopic}/can/emergency`;

    const responseTopicFormat: string = "/rms/ktp/dummy/response";
    const responsePathTopic: string = `${responseTopicFormat}/path`;
    const resposneGpsTopic: string = `${responseTopicFormat}/gps`;

    const onStraightClick = (): void => {
        onClickMqttPublish(mqttClient!, requestRouteToPoseTopic, straightJSON);
    }

    const onIntersection1Click = (): void => {
        onClickMqttPublish(mqttClient!, requestRouteToPoseTopic, intersection1JSON);
    }

    const onIntersection2Click = (): void => {
        onClickMqttPublish(mqttClient!, requestRouteToPoseTopic, intersection2JSON);
    }

    const onLoadingClick = (): void => {
        onClickMqttPublish(mqttClient!, requestRouteToPoseTopic, loadingJSON);
    }

    const onLanding1Click = (): void => {
        onClickMqttPublish(mqttClient!, requestRouteToPoseTopic, landing1JSON);
    }

    const onLanding2Click = (): void => {
        onClickMqttPublish(mqttClient!, requestRouteToPoseTopic, landing2JSON);
    }

    const onGPSShadow1Click = (): void => {
        onClickMqttPublish(mqttClient!, requestRouteToPoseTopic, gpsShadow1JSON);
    }

    const onGPSShadow2Click = (): void => {
        onClickMqttPublish(mqttClient!, requestRouteToPoseTopic, gpsShadow2JSON);
    }

    const onEmergencyStopClick = (): void => {
        onClickMqttPublish(mqttClient!, requestEmergencyTopic, emergencyStopJSON);
    }

    const onEmergencyResumeClick = (): void => {
        onClickMqttPublish(mqttClient!, requestEmergencyTopic, emergencyResumeJSON);
    }

    const handleResponseMQTTCallback = (mqttClient: MqttClient): void => {
        mqttClient.client.on("message", (topic: string, payload: Buffer, packet: IPublishPacket) => {
            if (topic === responsePathTopic) {
                console.info(`Path cb : ${JSON.stringify(JSON.parse(payload.toString()))}`);
                setPathData(JSON.parse(payload.toString()));
            } else if (topic === resposneGpsTopic) {
                setGpsData(JSON.parse(payload.toString()));
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
        handleResponseMQTTCallback(mqttClient);
    }, []);

    return (
        <div className="dash_board_container">
            <div className="top_component_container">
                <TopComponents />
            </div>
            <div className="map_component_container">
                <MapComponent pathData={pathData} gpsData={gpsData} />
            </div>
            <div className="request_component_container">
                <BlueSpaceRequestComponent
                    onStraightClick={onStraightClick}
                    onIntersection1Click={onIntersection1Click}
                    onIntersection2Click={onIntersection2Click}
                    onLoadingClick={onLoadingClick}
                    onLanding1Click={onLanding1Click}
                    onLanding2Click={onLanding2Click}
                    onGPSShadow1Click={onGPSShadow1Click}
                    onGPSShadow2Click={onGPSShadow2Click}
                    onEmergencyStopClick={onEmergencyStopClick}
                    onEmergencyResumeClick={onEmergencyResumeClick}
                />
            </div>
        </div>
    );
}