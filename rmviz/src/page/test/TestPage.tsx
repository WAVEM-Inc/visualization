import { Status, Wrapper } from "@googlemaps/react-wrapper";
import TestMapComponent from "../../components/map/TestMapComponent";
import TopComponents from "../../components/top/TopComponent";
import "../bluespace/BlueSpaceDashBoardPage.css";
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
import { onClickMqttPublish } from "../../utils/Utils";

const TestPage = () => {
    const [mqttClient, setMqttClient] = useState<MqttClient | undefined>();
    const [pathData, setPathData] = useState<any>(null);
    const [gpsData, setGpsData] = useState<any>(null);
    const [routeStatus, setRouteStatus] = useState<any>(null);
    const [odomEularData, setOdomEularData] = useState<any>(null);

    const blueSpaceCoord: naver.maps.LatLng = new naver.maps.LatLng(37.305985, 127.2401652);
    const requestTopicFormat: string = "/rms/ktp/dummy/request";
    const requestRouteToPoseTopic: string = `${requestTopicFormat}/route_to_pose`;
    const requestEmergencyTopic: string = `${requestTopicFormat}/can/emergency`;
    const requestGoalCancelTopic: string = `${requestTopicFormat}/goal/cancel`;
    const requestInitTopic: string = `${requestTopicFormat}/can/init`;

    const responseTopicFormat: string = "/rms/ktp/dummy/response";
    const responsePathTopic: string = `${responseTopicFormat}/path`;
    const resposneGpsTopic: string = `${responseTopicFormat}/gps`;
    const responseRouteStatusTopic: string = `${responseTopicFormat}/route/status`;
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

    const onInitClick = (): void => {
        onClickMqttPublish(mqttClient!, requestInitTopic, {
            "can_sign_tran_state": true
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
            } else if (topic === responseRouteStatusTopic) {
                setRouteStatus(JSON.parse(payload.toString()));
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
        mqttClient.subscribe(responseRouteStatusTopic);
        mqttClient.subscribe(responseOdomEularTopic);
        handleResponseMQTTCallback(mqttClient);
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
    const render = (status: Status) => {
        switch (status) {
            case Status.LOADING:
                return <>로딩중...</>;
            case Status.FAILURE:
                return <>에러 발생</>;
            case Status.SUCCESS:
                return <TestMapComponent
                center={blueSpaceCoord}
                pathData={pathData}
                gpsData={gpsData}
                odomEularData={odomEularData}
                routeStatus={routeStatus}
                onEmergencyStopClick={onEmergencyStopClick}
                onEmergencyResumeClick={onEmergencyResumeClick}
                onGoalCancelClick={onGoalCancelClick}
                onInitClick={onInitClick}
                />;
            default:
                return <>에러발생</>;
        }
    };

    return (
        <div className="dash_board_container">
            <div className="top_component_container">
                <TopComponents />
            </div>
            <div className="map_component_container">
                <Wrapper apiKey={"AIzaSyDYrrFBbCHPVWBrWm075rlxFJS8eDg-MPw"} render={render} libraries={['marker']} />
            </div>
            <div className="request_component_container">

            </div>
        </div>
    )
}

export default TestPage;