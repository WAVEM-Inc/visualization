import { IPublishPacket } from "mqtt/*";
import { useEffect, useState } from "react";
import MqttClient from "../../api/mqttClient";
import * as t201202JSON from "../../assets/json/bluespace/201-202.json";
import * as gpsShadow1JSON from "../../assets/json/bluespace/gps_shadow1.json";
import * as gpsShadow2JSON from "../../assets/json/bluespace/gps_shadow2.json";
import * as intersection1JSON from "../../assets/json/bluespace/intersection1.json";
import * as intersection2JSON from "../../assets/json/bluespace/intersection2.json";
import * as landing1JSON from "../../assets/json/bluespace/landing1.json";
import * as landing2JSON from "../../assets/json/bluespace/landing2.json";
import * as loadingJSON from "../../assets/json/bluespace/loading.json";
import * as straightJSON from "../../assets/json/bluespace/straight.json";
import * as emergencyResumeJSON from "../../assets/json/common/emergency_resume.json";
import * as emergencyStopJSON from "../../assets/json/common/emergency_stop.json";
import BlueSpaceRequestComponent from "../../components/bluespace/BlueSpaceRequestComponent";
import MapComponent from "../../components/map/MapComponents";
import TopComponents from "../../components/top/TopComponent";
import { onClickMqttPublish } from "../../utils/Utils";
import "./BlueSpaceDashBoardPage.css";

export default function BlueSpaceDashBoardPage() {
    const [mqttClient, setMqttClient] = useState<MqttClient | undefined>();
    const [pathData, setPathData] = useState<any>(null);
    const [gpsData, setGpsData] = useState<any>(null);
    const [routeToPoseStatus, setRouteToPoseStatus] = useState<any>(null);
    const [odomEularData, setOdomEularData] = useState<any>(null);

    const blueSpaceCoord: naver.maps.LatLng = new naver.maps.LatLng(37.305985, 127.2401652);
    const requestTopicFormat: string = "/rms/ktp/dummy/request";
    const requestRouteToPoseTopic: string = `${requestTopicFormat}/route_to_pose`;
    const requestEmergencyTopic: string = `${requestTopicFormat}/can/emergency`;
    const requestGoalCancelTopic: string = `${requestTopicFormat}/goal/cancel`;

    const responseTopicFormat: string = "/rms/ktp/dummy/response";
    const responsePathTopic: string = `${responseTopicFormat}/path`;
    const resposneGpsTopic: string = `${responseTopicFormat}/gps`;
    const responseRouteToPoseStatusTopic: string = `${responseTopicFormat}/route_to_pose/status`;
    const responseOdomEularTopic: string = `${responseTopicFormat}/odom/eular`;

    const on201202Click = (): void => {
        onClickMqttPublish(mqttClient!, requestRouteToPoseTopic, t201202JSON);
    }

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

    const onGoalCancelClick = (): void => {
        onClickMqttPublish(mqttClient!, requestGoalCancelTopic, {
            "cancel": true
        });
        setPathData(null);
    }

    const handleResponseMQTTCallback = (mqttClient: MqttClient): void => {
        mqttClient.client.on("message", (topic: string, payload: Buffer, packet: IPublishPacket) => {
            if (topic === responsePathTopic) {
                console.info(`Path cb : ${JSON.stringify(JSON.parse(payload.toString()))}`);
                setPathData(JSON.parse(payload.toString()));
            } else if (topic === resposneGpsTopic) {
                setGpsData(JSON.parse(payload.toString()));
            } else if (topic === responseRouteToPoseStatusTopic) {
                setRouteToPoseStatus(JSON.parse(payload.toString()));
            } else if (topic === responseOdomEularTopic) {
                setOdomEularData(JSON.parse(payload.toString()));
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
        mqttClient.subscribe(responseRouteToPoseStatusTopic);
        mqttClient.subscribe(responseOdomEularTopic);
        handleResponseMQTTCallback(mqttClient);
    }, []);

    useEffect(() => {
        if (routeToPoseStatus) {
            console.info(`routeToPoseStatus : ${JSON.stringify(routeToPoseStatus)}`);

            const index: number = routeToPoseStatus.index;
            const code: number = routeToPoseStatus.code;

            switch (code) {
                case 1:
                    alert(`[${index}] Goal Arrived`);
                    break;
                case 2:
                    alert(`[${index}] Goal Finished`);
                    break;
                default:
                    break;
            }
        } else {
            return;
        }

        return () => {
            setRouteToPoseStatus(null);
        }
    }, [routeToPoseStatus]);

    return (
        <div className="dash_board_container">
            <div className="top_component_container">
                <TopComponents />
            </div>
            <div className="map_component_container">
                <MapComponent center={blueSpaceCoord} pathData={pathData} gpsData={gpsData} odomEularData={odomEularData} />
            </div>
            <div className="request_component_container">
                <BlueSpaceRequestComponent
                    on201202Click={on201202Click}
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
                    onGoalCancelClick={onGoalCancelClick}
                />
            </div>
        </div>
    );
}