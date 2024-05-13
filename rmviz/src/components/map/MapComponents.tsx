import $ from "jquery";
import React, { useEffect, useRef, useState } from "react";
import MqttClient from "../../api/mqttClient";
import * as emergencyResumeJSON from "../../assets/json/common/emergency_resume.json";
import * as emergencyStopJSON from "../../assets/json/common/emergency_stop.json";
import { MapState } from "../../domain/map/MapDomain";
import { addGPSInitMarkerControl, initializeGPSInitMarker, initializeMap, initializeRobotMarker } from "../../service/MapService";
import { onClickMqttPublish } from "../../utils/Utils";
import "./MapComponents.css";

interface MapComponentProps {
    mqttClient: MqttClient;
    center: naver.maps.LatLng;
    state: MapState;
}

const MapComponent: React.FC<MapComponentProps> = ({
    mqttClient,
    center,
    state
}: MapComponentProps): React.ReactElement<any, any> | null => {
    const { naver }: Window & typeof globalThis = window;
    const mapRef: React.MutableRefObject<HTMLDivElement | null> = useRef<HTMLDivElement>(null);

    let map: naver.maps.Map | null = null;
    const [pathInfoContainer, setPathInfoContainer] = useState<HTMLElement | null>(null);
    const [pathInfoDiv, setPathInfoDiv] = useState<HTMLDivElement | null>(null);
    const [currRobotMarker, setCurrRobotMarker] = useState<naver.maps.Marker | null>(null);
    const [currGPSInitMarker, setCurrGPSInitMarker] = useState<naver.maps.Marker | null>(null);
    const [currentGps, setCurrentGps] = useState<any>({
        status: 0,
        service: 0,
        latitude: 0.0,
        longitude: 0.0
    });
    const [currentGpsFiltered, setCurrentGpsFiltered]: [any, React.Dispatch<any>] = useState<any>({
        status: 0,
        service: 0,
        latitude: 0.0,
        longitude: 0.0
    });
    const [currentOdomEular, setCurrentOdomEular]: [number | undefined, React.Dispatch<React.SetStateAction<number | undefined>>] = useState<number>();
    const [currentRouteStatus, setCurrentRouteStatus]: [any, React.Dispatch<any>] = useState<any | null>(null);

    let pathMarkerArray: Array<naver.maps.Marker> = [];
    let pathInfoWindowarray: Array<naver.maps.InfoWindow> = [];

    const [defaultZoom, setDefaultZoom]: [number, React.Dispatch<React.SetStateAction<number>>] = useState<number>(20);

    const requestTopicFormat: string = "/rms/ktp/dummy/request";
    const requestEmergencyTopic: string = `${requestTopicFormat}/can/emergency`;
    const requestGoalCancelTopic: string = `${requestTopicFormat}/goal/cancel`;
    const requestGPSInitTopic: string = `${requestTopicFormat}/gps/init`;
    const requestCanInitTopic: string = `${requestTopicFormat}/can/init`;
    const requestPathRenewTopic: string = `${requestTopicFormat}/path/renew`;

    const addPathMarker: Function = (node: any, is_start: boolean, is_end: boolean): any => {
        console.info(`addPathMarker node : ${JSON.stringify(node)}`);

        let iconUrl: string = "";

        if (is_start && !is_end) {
            iconUrl = process.env.PUBLIC_URL + "../marker_start.png";
        } else if (!is_start && is_end) {
            iconUrl = process.env.PUBLIC_URL + "../marker_arrive.png";
        } else {
            iconUrl = process.env.PUBLIC_URL + "../marker_landmark.png";
        }

        const nodeTitleOpts: any = {
            id: node.nodeId.split("-")[2],
            kind: node.kind,
            heading: node.heading,
            drivingOption: node.drivingOption,
            direction: node.direction
        }

        const nodeTitle: string = `${nodeTitleOpts.id}/${nodeTitleOpts.kind}/${node.heading}/${node.drivingOption}/${node.direction}`;

        const marker: naver.maps.Marker = new naver.maps.Marker({
            position: new naver.maps.LatLng(node.position.latitude, node.position.longitude),
            map: map,
            title: nodeTitle,
            icon: {
                url: iconUrl,
                size: new naver.maps.Size(30, 30),
                scaledSize: new naver.maps.Size(30, 30),
                origin: new naver.maps.Point(0, 0),
                anchor: new naver.maps.Point(12, 34)
            }
        });

        return marker;
    }

    const getClickHandler: Function = (seq: number): Function => {
        let isOpened: number = 0;
        return function (e: any) {
            const marker: naver.maps.Marker = pathMarkerArray[seq];
            const infoWindow: naver.maps.InfoWindow = pathInfoWindowarray[seq];

            if (isOpened == 0) {
                infoWindow.open(map!, marker);
                isOpened++;
            } else {
                infoWindow.close();
                isOpened = 0;
            }
        }
    }

    const drawPathMarker: Function = (): void => {
        if (state.path) {
            const nodeList: Array<any> = Array.from(state.path);

            const uniqueNodeIds: Set<string> = new Set<string>();
            for (const node of nodeList) {
                uniqueNodeIds.add(node.nodeId.split("-")[2]);
            }

            for (const nodeId of uniqueNodeIds) {
                const node: any = nodeList.find((node) => node.nodeId.split("-")[2] === nodeId);
                if (node) {
                    const currentNodeIndex: number = nodeList.indexOf(node);
                    const nextNode: any = nodeList[currentNodeIndex + 1];
                    const nextNodeIndex: number = nodeList.indexOf(nextNode);

                    const isFirstNode = currentNodeIndex === 0;
                    const isLastNode = currentNodeIndex === nodeList.length - 1;

                    if (isFirstNode) {
                        pathMarkerArray.push(addPathMarker(node, true, false));
                    } else if (isLastNode) {
                        pathMarkerArray.push(addPathMarker(node, false, true));
                    } else {
                        pathMarkerArray.push(addPathMarker(node, false, false));
                    }

                    const pathInfoContainer: HTMLElement | null = document.getElementById("path_info_container");

                    let startNodeIconUrl: string = "";
                    let endNodeIconUrl: string = "";

                    if (!isLastNode) {
                        if (isFirstNode) {
                            startNodeIconUrl = process.env.PUBLIC_URL + "../marker_start.png";
                        } else if (isLastNode) {
                            startNodeIconUrl = process.env.PUBLIC_URL + "../marker_arrive.png";
                        } else {
                            startNodeIconUrl = process.env.PUBLIC_URL + "../marker_landmark.png";
                        }

                        if (nextNodeIndex === nodeList.length - 1) {
                            endNodeIconUrl = process.env.PUBLIC_URL + "../marker_arrive.png";
                        } else {
                            endNodeIconUrl = process.env.PUBLIC_URL + "../marker_landmark.png";
                        }
                    } else {
                        return;
                    }

                    const startNodeTitleOpts: any = {
                        id: node.nodeId.split("-")[2],
                        kind: node.kind,
                        heading: node.heading,
                        drivingOption: node.drivingOption,
                        direction: node.direction
                    }

                    const endNodeTitleOpts: any = {
                        id: nextNode.nodeId.split("-")[2],
                        kind: nextNode.kind,
                        heading: nextNode.heading,
                        drivingOption: nextNode.drivingOption,
                        direction: nextNode.direction
                    }

                    const pathInfoDiv: HTMLDivElement = document.createElement("div");
                    pathInfoDiv.className = "path_info";

                    const startNodeDiv: HTMLDivElement = document.createElement("div");
                    startNodeDiv.className = "path_start_node";

                    const startNodeIcon: HTMLImageElement = document.createElement("img");
                    startNodeIcon.src = startNodeIconUrl;
                    startNodeIcon.className = "node_icon start_node_icon";

                    const startNodeInfo: HTMLDivElement = document.createElement("div");
                    startNodeInfo.className = "node_info start_node_info";
                    startNodeInfo.textContent = `[ ${startNodeTitleOpts.id} ]`;

                    startNodeDiv.appendChild(startNodeIcon);
                    startNodeDiv.appendChild(startNodeInfo);

                    const pathDirectionDiv: HTMLDivElement = document.createElement("div");
                    pathDirectionDiv.className = "path_direction";

                    const pathDirectionIcon: HTMLImageElement = document.createElement("img");
                    pathDirectionIcon.src = process.env.PUBLIC_URL + "../arrow-right.png";
                    pathDirectionIcon.className = "node_icon";

                    const pathDirectionDivContent: HTMLDivElement = document.createElement("div");
                    pathDirectionDivContent.className = "path_direction_content";
                    pathDirectionDivContent.textContent = `${startNodeTitleOpts.drivingOption} - ${startNodeTitleOpts.direction}`;

                    pathDirectionDiv.appendChild(pathDirectionIcon);
                    pathDirectionDiv.appendChild(pathDirectionDivContent);

                    const endNodeDiv: HTMLDivElement = document.createElement("div");
                    endNodeDiv.className = "path_end_node";

                    const endNodeIcon: HTMLImageElement = document.createElement("img");
                    endNodeIcon.src = endNodeIconUrl;
                    endNodeIcon.className = "node_icon end_node_icon";

                    const endNodeInfo: HTMLDivElement = document.createElement("div");
                    endNodeInfo.className = "node_info end_node_info";
                    endNodeInfo.textContent = `[ ${endNodeTitleOpts.id} ]`;

                    endNodeDiv.appendChild(endNodeIcon);
                    endNodeDiv.appendChild(endNodeInfo);

                    pathInfoDiv.appendChild(startNodeDiv);
                    pathInfoDiv.appendChild(pathDirectionDiv);
                    pathInfoDiv.appendChild(endNodeDiv);
                    setPathInfoDiv(pathInfoDiv);
                    pathInfoContainer!.appendChild(pathInfoDiv);
                }
            }
        }
    }

    const drawPathPolyline: Function = (): void => {
        for (var i = 0, ii = pathMarkerArray.length; i < ii; i++) {
            naver.maps.Event.addListener(pathMarkerArray[i], "click", getClickHandler(i));
        }

        let path: Array<any> = [];
        for (const pathMarker of pathMarkerArray) {
            const contentString: string = [
                '<div class="iw_inner">',
                `   <h3>ID : ${pathMarker!.getTitle().split("/")[0]}</h3>`,
                `   <p>종류 : ${pathMarker!.getTitle().split("/")[1]}</p>`,
                `   <p>진출 각도 : ${pathMarker!.getTitle().split("/")[2]}</p>`,
                `   <p>주행 옵션 : ${pathMarker!.getTitle().split("/")[3]}</p>`,
                `   <p>주행 방향 : ${pathMarker!.getTitle().split("/")[4]}</p>`,
                `   <p>경도 : ${pathMarker!.getPosition().x}</p>`,
                `   <p>위도 : ${pathMarker!.getPosition().y}</p>`,
                '</div>'
            ].join('');

            const infoWindow: naver.maps.InfoWindow = new naver.maps.InfoWindow({
                content: contentString
            });

            pathInfoWindowarray.push(infoWindow);

            for (var i = 0, ii = pathMarkerArray.length; i < ii; i++) {
                naver.maps.Event.addListener(pathMarkerArray[i], "click", getClickHandler(i));
            }
            path.push(pathMarker.getPosition());
        }

        const polyline: naver.maps.Polyline = new naver.maps.Polyline({
            map: map,
            path: path,
            clickable: true,
            strokeColor: "white",
            strokeStyle: "line",
            strokeLineCap: "round",
            strokeLineJoin: "round",
            strokeOpacity: 1.0,
            strokeWeight: 5.0
        });
    }

    const changeMapCenter = (coord: naver.maps.Coord): void => {
        map?.setCenter(coord);
    }

    const onGPSInitClick = (): void => {
        if (currGPSInitMarker?.getMap()) {
            console.info(`GPS init Marker coord : ${JSON.stringify(currGPSInitMarker?.getPosition())}`);
            onClickMqttPublish(mqttClient!, requestGPSInitTopic, currGPSInitMarker?.getPosition());
        } else {
            alert("GPS 초기화 마커를 배치해주세요.");
            return;
        }
    }

    const onCanInitClick = (): void => {
        onClickMqttPublish(mqttClient!, requestCanInitTopic, {
            "can_sign_tran_state": true
        });
    }

    const onPathRenewClick = (): void => {
        onClickMqttPublish(mqttClient!, requestPathRenewTopic, {});
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
        state.path = null;
        state.routeStatus = null;
    }

    useEffect((): void => {
        if (mapRef.current && naver && !map) {
            map = initializeMap(mapRef.current, center);
        } else return;

        if (mapRef.current && naver && map) {
            const robotMarker: naver.maps.Marker = initializeRobotMarker(map);
            setCurrRobotMarker(robotMarker);

            const gpsInitMarker: naver.maps.Marker = initializeGPSInitMarker(map);
            gpsInitMarker.setMap(null);

            addGPSInitMarkerControl(map, gpsInitMarker);
            setCurrGPSInitMarker(gpsInitMarker);

            naver.maps.Event.addListener(map, "click", function (e: any) {
                if (gpsInitMarker.getMap()) {
                    const coord: any = e.coord;
                    gpsInitMarker.setPosition(coord);
                }
            });
        }

        if (mapRef.current && naver && map) {
            if (state.path) {
                console.info(`state.path : ${JSON.stringify(state.path)}`);

                if (pathInfoDiv) {
                    if (pathInfoContainer) {
                        while (pathInfoContainer.firstChild) {
                            pathInfoContainer.removeChild(pathInfoContainer.firstChild);
                        }
                    }
                    setPathInfoDiv(null);
                }

                drawPathMarker();
                drawPathPolyline();
                setDefaultZoom(21);

                if (currRobotMarker) {
                    changeMapCenter(currRobotMarker!.getPosition());
                }
            }
        } else return;
    }, [naver, map, state.path]);

    useEffect((): void => {
        if (state.gps) {
            const gpsStatus: any | undefined = state.gps.status;
            setCurrentGps({
                status: gpsStatus?.status,
                service: gpsStatus?.service,
                longitude: parseFloat(state.gps.longitude?.toFixed(7)),
                latitude: parseFloat(state.gps.latitude?.toFixed(7))
            });
            if (currRobotMarker) {
                if (state.gps.longitude != 0.0 && state.gps.latitude != 0.0) {
                    currRobotMarker!.setPosition(new naver.maps.LatLng(state.gps.latitude, state.gps.longitude));
                } else return;
            }
        }
    }, [state.gps]);

    useEffect((): void => {
        if (state.gpsFiltered) {
            const gpsFilteredStatus: any | undefined = state.gpsFiltered.status;
            setCurrentGpsFiltered({
                status: gpsFilteredStatus?.status,
                service: gpsFilteredStatus?.service,
                longitude: parseFloat(state.gpsFiltered.longitude?.toFixed(7)),
                latitude: parseFloat(state.gpsFiltered.latitude?.toFixed(7))
            });
        }
    }, [state.gpsFiltered]);

    useEffect((): void => {
        if (state.odomEular) {
            const poseOrientation: any | undefined = state.odomEular.pose?.orientation;
            setCurrentOdomEular(parseFloat(poseOrientation?.y.toFixed(2)) | 0.0);
        }

        if (currentOdomEular) {
            const angle: number = 360 - currentOdomEular!;
            $("div[title|='RobotCurrentPos'").css("transform", `rotate(${angle}deg)`);
        }

    }, [state.odomEular]);

    useEffect((): void => {
        if (state.routeStatus) {
            console.info(`currentRouteStatus : ${JSON.stringify(state.routeStatus)}`);

            let driving_flag: boolean = false;
            if (state.routeStatus._is_driving) {
                driving_flag = true;
            } else {
                driving_flag = false;
            }

            let status: string = "";
            switch (state.routeStatus._status_code) {
                case 0:
                    status = "출발";
                    break;
                case 1:
                    status = "경유지 도착";
                    break;
                case 2:
                    status = "주행 완료";
                    break;
                case 3:
                    status = "주행 서버 미작동";
                    alert("주행 서버가 구동 중이지 않습니다.");
                    break;
                case 4:
                    status = "주행 진행 중";
                    alert("주행이 이미 진행 중입니다.");
                    break;
                case 5:
                    status = "주행 취소";
                    break;
                default:
                    break;
            }

            const node_info: any = state.routeStatus._node_info;

            if (node_info) {
                const currRouteStatus: any = {
                    driving_flag: driving_flag,
                    status: status,
                    node_info: node_info!
                };

                setCurrentRouteStatus(currRouteStatus);
            }
        }
    }, [state.routeStatus]);

    useEffect(() => {
        setPathInfoContainer(document.getElementById("path_info_container"));
        return () => {
            if (map) {
                setCurrRobotMarker(null);
                setPathInfoDiv(null);
                map.destroy();
                map = null;
            }
        }
    }, []);

    return (
        <div className={"map_components"}>
            <div className={"map_container"}>
                <div ref={mapRef} id={"map"} />
                <div id="path_info_container" className="path_info_container" />
                <div className="data_container">
                    <div className={"gps_data_container"}>
                        <div className="data_title">
                            GPS
                        </div>
                        <div className="data_data">
                            Status : {currentGps.status}
                            <br></br>
                            Service : {currentGps.service}
                            <br></br>
                            경위도 : {currentGps.longitude}, {currentGps.latitude}
                            <br></br>
                            필터링 경위도 : {currentGpsFiltered.longitude}, {currentGpsFiltered.latitude}
                            <br></br>
                            오차 : {Math.abs(currentGps.longitude - currentGpsFiltered.longitude).toFixed(7)}, {Math.abs(currentGps.latitude - currentGpsFiltered.latitude).toFixed(7)}
                        </div>
                    </div>
                    <div className={"odom_eular_data_container"}>
                        <div className="data_title">
                            차량 각도
                        </div>
                        <div className="data_data">
                            {currentOdomEular}
                        </div>
                    </div>
                    <div className="route_request_btn_container">
                        <button className={"route_btn_request route_btn_gps_init"} onClick={onGPSInitClick}>GPS 초기화</button>
                        <button className={"route_btn_request route_btn_can_init"} onClick={onCanInitClick}>CAN 초기화</button>
                        <button className={"route_btn_request route_btn_path_renew"} onClick={onPathRenewClick}>경로 갱신</button>
                        <button className={"route_btn_request route_btn_emergency_stop"} onClick={onEmergencyStopClick}>비상 정지</button>
                        <button className={"route_btn_request route_btn_emergency_stop"} onClick={onGoalCancelClick}>주행 취소</button>
                        <button className={"route_btn_request route_btn_emergency_resume"} onClick={onEmergencyResumeClick}>재개</button>
                    </div>
                </div>
            </div>
        </div>
    );
};

export default MapComponent;