import { Wrapper } from "@googlemaps/react-wrapper";
import React, { useEffect, useState } from "react";
import ReactModal from "react-modal";
import MqttClient from "../../api/mqttClient";
import * as emergencyStopJSON from "../../assets/json/common/emergency_stop.json";
import { MapState } from "../../domain/map/MapDomain";
import { addDetectionRangePolygon, addPathMarker, addPathPolyline, changeMapCenter, initializeKECDBorderLine, initializeMap, initializeRobotMarker, updateRobotMakerIcon } from "../../service/map/MapService";
import { onClickMqttPublish } from "../../utils/Utils";
import "./GoogleMapComponent.css";
import GoogleMapPathComponent from "./GoogleMapPathComponent";

interface GoogleMapComponentProps {
    mqttClient: MqttClient;
    state: MapState;
}

const GoogleMapComponent: React.FC<GoogleMapComponentProps> = ({
    mqttClient,
    state
}: GoogleMapComponentProps) => {
    const [googleMap, setGoogleMap] = useState<google.maps.Map>();
    const [pathInfoContainer, setPathInfoContainer] = useState<HTMLElement | null>(null);
    const [pathInfoDiv, setPathInfoDiv] = useState<HTMLDivElement | null>(null);
    const [spathMarkerArray, setPathMarkerArray] = useState<Array<google.maps.Marker>>([]);
    const [pathPolyLine, setPathPolyLine] = useState<google.maps.Polyline | null>(null);
    const [detectionRagnePolygon, setDetectionRagnePolygon] = useState<Array<google.maps.Polygon>>([]);
    const [isPathSelectModalOpen, setIsPathSelectModalOpen] = useState<boolean>(false);

    let pathMarkerArray: Array<google.maps.Marker> = [];
    let pathInfoWindowarray: Array<google.maps.InfoWindow> = [];

    const kecCoord: google.maps.LatLng = new google.maps.LatLng(36.11434, 128.3690);
    const [currRobotMarker, setCurrRobotMarker] = useState<google.maps.Marker | null>(null);
    const [currGPSInitMarker, setCurrGPSInitMarker] = useState<google.maps.Marker | null>(null);
    const [currentGps, setCurrentGps] = useState<any>({
        status: 0,
        service: 0,
        latitude: 0.0,
        longitude: 0.0
    });
    const [currentOdomEular, setCurrentOdomEular] = useState<number>();
    const [currentRouteStatus, setCurrentRouteStatus] = useState<any | null>(null);
    const [isCommandRouteSwitchOn, setIsCommandRouteSwitchOn] = useState<boolean>(false);

    const requestTopicFormat: string = "/rmviz/request";
    const requestEmergencyTopic: string = `${requestTopicFormat}/can/emergency`;
    const requestGoalCancelTopic: string = `${requestTopicFormat}/goal/cancel`;
    const requestGPSInitTopic: string = `${requestTopicFormat}/gps/init`;
    const requestCanInitTopic: string = `${requestTopicFormat}/can/init`;
    const requestPathRenewTopic: string = `${requestTopicFormat}/path/renew`;
    const requestTaskTopic: string = `${requestTopicFormat}/task`;

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

                    const isFirstNode: boolean = currentNodeIndex === 0;
                    const isLastNode: boolean = currentNodeIndex === nodeList.length - 1;
                    const isIntersectionNode: boolean = node.kind === "intersection";

                    if (isFirstNode) {
                        pathMarkerArray.push(addPathMarker(googleMap, node, true, false));
                    } else if (isLastNode) {
                        pathMarkerArray.push(addPathMarker(googleMap, node, false, true));
                    } else {
                        pathMarkerArray.push(addPathMarker(googleMap, node, false, false));
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
                            if (isIntersectionNode) {
                                startNodeIconUrl = process.env.PUBLIC_URL + "../marker_intersection.png";
                            } else {
                                startNodeIconUrl = process.env.PUBLIC_URL + "../marker_landmark.png";
                            }
                        }

                        if (nextNodeIndex === nodeList.length - 1) {
                            endNodeIconUrl = process.env.PUBLIC_URL + "../marker_arrive.png";
                        } else {
                            if (isIntersectionNode) {
                                endNodeIconUrl = process.env.PUBLIC_URL + "../marker_intersection.png";
                            } else {
                                endNodeIconUrl = process.env.PUBLIC_URL + "../marker_landmark.png";
                            }
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

                    const pathHeadingDivContent: HTMLDivElement = document.createElement("div");
                    pathHeadingDivContent.className = "path_heading_content";
                    pathHeadingDivContent.textContent = `${startNodeTitleOpts.heading}°`;

                    pathDirectionDiv.appendChild(pathDirectionIcon);
                    pathDirectionDiv.appendChild(pathDirectionDivContent);
                    pathDirectionDiv.appendChild(pathHeadingDivContent);

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
            setPathMarkerArray(pathMarkerArray);
        }
    }

    const flushPath = (): void => {
        spathMarkerArray.forEach(marker => marker.setMap(null));
        pathMarkerArray = [];
        setPathMarkerArray(pathMarkerArray);

        pathInfoWindowarray.forEach(infoWindow => infoWindow.close());
        pathInfoWindowarray = [];

        if (pathPolyLine) {
            pathPolyLine.setMap(null);
            setPathPolyLine(null);
        }

        for (const d of detectionRagnePolygon) {
            d.setMap(null);
        }
        setDetectionRagnePolygon([]);

        if (pathInfoContainer) {
            while (pathInfoContainer.firstChild) {
                pathInfoContainer.removeChild(pathInfoContainer.firstChild);
            }
        }
    }

    const openPathSelectModal = (): void => {
        setIsPathSelectModalOpen(true);
    }

    const closePathSelectModal = (): void => {
        setIsPathSelectModalOpen(false);
    }

    const onPathSelectClick = (): void => {
        mqttClient.publish("/rmviz/request/path/select", JSON.stringify({}));
        openPathSelectModal();
    }

    const onPathRenewClick = (): void => {
        onClickMqttPublish(mqttClient!, requestPathRenewTopic, {});
    }

    const onEmergencyStopClick = (): void => {
        onClickMqttPublish(mqttClient!, requestEmergencyTopic, emergencyStopJSON);
    }

    const onGoalCancelClick = (): void => {
        onClickMqttPublish(mqttClient!, requestGoalCancelTopic, {
            "cancel": true
        });
        state.path = null;
        state.routeStatus = null;
        flushPath();
    }

    const onCommandRouteSwtichClick: React.MouseEventHandler<HTMLDivElement> = (): void => {
        setIsCommandRouteSwitchOn(!isCommandRouteSwitchOn);

        if (isCommandRouteSwitchOn === true) {
            localStorage.setItem("isEnableToCommandRoute?", "false");
        } else {
            localStorage.setItem("isEnableToCommandRoute?", "true");
        }
    }

    useEffect((): void => {
        if (state.gps) {
            if (!isNaN(state.gps.latitude) && !isNaN(state.gps.longitude)) {
                const gpsStatus: any | undefined = state.gps.status;
                setCurrentGps({
                    status: gpsStatus?.status,
                    service: gpsStatus?.service,
                    longitude: parseFloat(state.gps.longitude?.toFixed(7)),
                    latitude: parseFloat(state.gps.latitude?.toFixed(7))
                });

                if (currRobotMarker) {
                    if (state.gps.longitude !== 0.0 && state.gps.latitude !== 0.0) {
                        currRobotMarker!.setPosition(new google.maps.LatLng(state.gps.latitude, state.gps.longitude));
                    } else return;
                }
            }
        }
    }, [state.gps]);

    useEffect((): void => {
        if (state.odomEular) {
            const poseOrientation: any | undefined = state.odomEular.pose?.orientation;
            setCurrentOdomEular(parseFloat(poseOrientation?.y.toFixed(2)) | 0.0);
        }

        if (currentOdomEular) {
            const angle: number = 360 - currentOdomEular;

            if (currRobotMarker) {
                updateRobotMakerIcon(currRobotMarker, angle);
            }
        }
    }, [state.odomEular]);

    useEffect((): void => {
        if (googleMap) {
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

                flushPath();
                drawPathMarker();

                setDetectionRagnePolygon(addDetectionRangePolygon(googleMap, Array.from(state.path)));
                setPathPolyLine(addPathPolyline(googleMap, pathMarkerArray, pathInfoWindowarray));

                if (currRobotMarker) {
                    changeMapCenter(googleMap, currRobotMarker!.getPosition()!);
                }
            }
        } else return;
    }, [state.path]);

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
                    status = "주행 서버가 구동되지 않았습니다.";
                    alert(`${status}`);
                    break;
                case 4:
                    status = "주행 진행 중";
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

    useEffect((): void => {
        if (state.cmdVel) {
            console.info(`cmdVel : ${JSON.stringify(state.cmdVel)}`);

            if (currentGps) {
                // if (state.cmdVel.linear.x > 0.0) {
                //     const circle: google.maps.Circle = new google.maps.Circle({
                //         map: googleMap,
                //         center: new google.maps.LatLng(currentGps.latitude, currentGps.longitude),
                //         strokeColor: "#FF0000",
                //         strokeOpacity: 0.8,
                //         strokeWeight: 2,
                //         fillColor: "#FF0000",
                //         fillOpacity: 0.35,
                //         radius: 1
                //     });
                // }
            }
        }
    }, [state.cmdVel]);

    useEffect((): void => {
        if (googleMap) {
            const robotMarker: google.maps.Marker = initializeRobotMarker(googleMap);
            setCurrRobotMarker(robotMarker);

            initializeKECDBorderLine(googleMap);
            // 36.1138065627487, 128.3674145275724
            // 36.113806562688765, 128.3675346220269
            // 36.11370954463808, 128.3675346220269

            // const newCoords = calculateOffset(36.11370954461467, 128.36745956308556, 19.7, 240);
            // console.info(`newCoord : ${JSON.stringify(newCoords)}`);

            // const angle = calculateBearing(36.1137154, 128.368048, 36.1137132, 128.3677302);
            // console.info(`angle : ${angle}`);
            // alert(`angle : ${angle}`);

            // new google.maps.Marker({
            //     map: googleMap,
            //     position: new google.maps.LatLng(newCoords.lat, newCoords.lon)
            // });
        }
    }, [googleMap]);

    useEffect(() => {
        const mapElement: HTMLElement | null = document.getElementById("map");

        if (mapElement) {
            const mapInstance: google.maps.Map = initializeMap(mapElement, kecCoord);

            setGoogleMap(mapInstance);
            setPathInfoContainer(document.getElementById("path_info_container"));
        }
        localStorage.setItem("isEnableToCommandRoute?", "false");

        return (() => {
            if (googleMap) {
                flushPath();
                googleMap.unbindAll();
            }
        });
    }, []);

    return (
        <div className={"map_components"}>
            <div className={"map_container"}>
                <div id="map" />
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
                        </div>
                    </div>
                    <div className={"odom_eular_data_container"}>
                        <div className="data_title">
                            차량 각도
                        </div>
                        <div className="data_data">
                            {currentOdomEular}°
                        </div>
                    </div>
                    <div className="route_request_btn_container">
                        <button className={"route_btn_request route_btn_path_select"} onClick={onPathSelectClick}>
                            <img src={process.env.PUBLIC_URL + "../marker_route.png"} />
                            <p>경로 선택</p>
                        </button>
                        <button className={"route_btn_request route_btn_path_renew"} onClick={onPathRenewClick}>
                            <img src={process.env.PUBLIC_URL + "../marker_refresh.png"} />
                            <p>경로 갱신</p>
                        </button>
                        <button className={"route_btn_request route_btn_path_cancel"} onClick={onGoalCancelClick}>
                            <img src={process.env.PUBLIC_URL + "../marker_route_cancel.png"} />
                            <p>경로 취소</p>
                        </button>
                        <button className={"route_btn_request route_btn_emergency_stop"} onClick={onEmergencyStopClick}>
                            <img src={process.env.PUBLIC_URL + "../marker_stop.png"} />
                            <p>비상 정지</p>
                        </button>
                    </div>
                </div>
                <ReactModal
                    id="route_path_select_modal"
                    className={"route_path_select_modal"}
                    isOpen={isPathSelectModalOpen}
                    onRequestClose={closePathSelectModal}
                >
                    <div className="route_path_select_modal_container">
                        <div className="route_path_select_title">
                            <img src={process.env.PUBLIC_URL + "../marker_route.png"} />
                            <p>경로 선택</p>
                        </div>
                        <div className="route_path_select_close_btn">
                            <img src={process.env.PUBLIC_URL + "../btn_close.png"} onClick={() => { closePathSelectModal() }}></img>
                        </div>
                        <Wrapper apiKey={`${process.env.GOOGLE_MAP_API_KEY}`}>
                            <GoogleMapPathComponent
                                mqttClient={mqttClient!}
                                state={state}
                            />
                        </Wrapper>
                    </div>
                </ReactModal>
            </div>
        </div>
    );
}

export default GoogleMapComponent;