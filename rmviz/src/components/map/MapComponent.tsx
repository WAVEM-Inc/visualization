import { Wrapper } from "@googlemaps/react-wrapper";
import mqtt from "mqtt/*";
import React, { useEffect, useState } from "react";
import ReactModal from "react-modal";
import Rqtt from "../../api/application/rqtt";
import { RTM_TOPIC_FORMAT } from "../../api/domain/common.constants";
import * as emergencyStopJSON from "../../assets/json/common/emergency_stop.json";
import * as controlMoveToDestJSON from "../../assets/json/control_movetodest.json";
import * as controlMsCancelJSON from "../../assets/json/control_mscancel.json";
import * as controlMsCompleteNoReturnJSON from "../../assets/json/control_mscomplete_no_return.json";
import * as controlMsCompleteReturnJSON from "../../assets/json/control_mscomplete_return.json";
import * as missionDeliveryJSON from "../../assets/json/mission_delivering.json";
import * as missionReturningJSON from "../../assets/json/mission_returning.json";
import { addDetectionRangePolygon, addPathMarker, addPathPolyline, changeMapCenter, initializeKECDBorderLine, initializeMap, initializeRobotMarker, recordNavigatedPathCircle, updateRobotMakerIcon } from "../../service/map/MapService";
import { onClickMqttPublish } from "../../utils/Utils";
import PathComponent from "../path/PathComponent";
import "./MapComponent.css";

interface MapComponentProps {
    rqtt: Rqtt,
    rqttC: mqtt.MqttClient;
}

const MapComponent: React.FC<MapComponentProps> = ({
    rqtt,
    rqttC
}: MapComponentProps) => {
    const [path, setPath] = useState<any | null>(null);
    const [routeStatus, setRouteStatus] = useState<any | null>(null);
    const [gps, setGps] = useState<any | null>(null);
    const [odomEular, setOdomEular] = useState<any | null>(null);
    const [cmdVel, setCmdVel] = useState<any | null>(null);

    const [googleMap, setGoogleMap] = useState<google.maps.Map>();
    const [isDriving, setIsDriving] = useState<boolean>(false);
    const [pathInfoContainer, setPathInfoContainer] = useState<HTMLElement | null>(null);
    const [pathInfoDiv, setPathInfoDiv] = useState<HTMLDivElement | null>(null);
    const [_pathMarkerArray, setPathMarkerArray] = useState<Array<google.maps.Marker>>([]);
    const [pathPolyLine, setPathPolyLine] = useState<google.maps.Polyline | null>(null);
    const [_pathCircleArray, setPathCircleArray] = useState<Array<google.maps.Circle>>([]);
    const [detectionRagnePolygon, setDetectionRagnePolygon] = useState<Array<google.maps.Polygon>>([]);
    const [isPathSelectModalOpen, setIsPathSelectModalOpen] = useState<boolean>(false);
    const [isMissionSelectModalOpen, setIsMissionSelectModalOpen] = useState<boolean>(false);

    let pathMarkerArray: Array<google.maps.Marker> = [];
    let pathInfoWindowarray: Array<google.maps.InfoWindow> = [];

    const kecCoord: google.maps.LatLng = new google.maps.LatLng(36.11434, 128.3690);
    const [currRobotMarker, setCurrRobotMarker] = useState<google.maps.Marker | null>(null);
    const [currentGps, setCurrentGps] = useState<any>({
        status: 0,
        service: 0,
        latitude: 0.0,
        longitude: 0.0
    });
    const [currentOdomEular, setCurrentOdomEular] = useState<number>();
    const [currentRouteStatus, setCurrentRouteStatus] = useState<any | null>(null);
    const [currentCmdVel, setCurrentCmdVel] = useState<any>({});
    const [cmdVelFlag, setCmdVelFlag] = useState<boolean>(false);

    const requestTopicFormat: string = "net/wavem/rms/rqtt/mtr";
    const requestEmergencyTopic: string = `${requestTopicFormat}/drive/can/emergency`;
    const requestGoalCancelTopic: string = `${requestTopicFormat}/goal/cancel`;
    const requestPathRenewTopic: string = `${requestTopicFormat}/path/renew`;
    const requestPathSelectTopic: string = `${requestTopicFormat}/path/select`;
    const requestTaskTopic: string = `${requestTopicFormat}/task`;

    const drawPathMarker: Function = (): void => {
        if (path) {
            const nodeList: Array<any> = Array.from(path);

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

    const flushPath: Function = (): void => {
        _pathMarkerArray.forEach(marker => marker.setMap(null));
        pathMarkerArray = [];
        setPathMarkerArray(pathMarkerArray);

        pathInfoWindowarray.forEach(infoWindow => infoWindow.close());
        pathInfoWindowarray = [];

        _pathCircleArray.forEach(circle => circle.setMap(null));
        setPathCircleArray([]);

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

    const openMissionSelectModal = (): void => {
        setIsMissionSelectModalOpen(true);
    }

    const closeMissionSelectMoal = (): void => {
        setIsMissionSelectModalOpen(false);
    }

    const onPathSelectClick = (): void => {
        rqtt.publish(rqttC, requestPathSelectTopic, JSON.stringify({}));
        openPathSelectModal();
    }

    const onMissionSelectClick = (): void => {
        openMissionSelectModal();
    }

    const onPathRenewClick = (): void => {
        onClickMqttPublish(rqtt, rqttC, requestPathRenewTopic, {});
    }

    const onEmergencyStopClick = (): void => {
        onClickMqttPublish(rqtt, rqttC, requestEmergencyTopic, emergencyStopJSON);
    }

    const onGoalCancelClick = (): void => {
        onClickMqttPublish(rqtt, rqttC, requestGoalCancelTopic, {
            "cancel": true
        });
        setPath(null);
        setRouteStatus(null);
        flushPath();
    }

    const onTaskClick = (taskJSON: any): void => {
        console.info(`Task : ${JSON.stringify(taskJSON)}`);
        onClickMqttPublish(rqtt, rqttC, requestTaskTopic, taskJSON);
    }

    const pathCallback: (message: any) => void = (message: any): void => {
        setPath(message);
    }

    const routeStatusCallback: (message: any) => void = (message: any): void => {
        setRouteStatus(message);
    }

    const gpsCallback: (message: any) => void = (message: any): void => {
        setGps(message);
    }

    const odomEularCallback: (message: any) => void = (message: any): void => {
        setOdomEular(message);
    }

    const cmdVelCallback: (message: any) => void = (message: any): void => {
        setCmdVel(message);
    }

    useEffect((): void => {
        if (gps) {
            if (!isNaN(gps.latitude) && !isNaN(gps.longitude)) {
                const gpsStatus: any | undefined = gps.status;
                setCurrentGps({
                    status: gpsStatus?.status,
                    service: gpsStatus?.service,
                    longitude: parseFloat(gps.longitude?.toFixed(7)),
                    latitude: parseFloat(gps.latitude?.toFixed(7))
                });

                if (currRobotMarker) {
                    if (gps.longitude !== 0.0 && gps.latitude !== 0.0) {
                        currRobotMarker!.setPosition(new google.maps.LatLng(gps.latitude, gps.longitude));

                        if (currRobotMarker.getPosition() && isDriving) {
                            changeMapCenter(googleMap, currRobotMarker.getPosition());
                        }
                    } else return;
                }
            }
        }
    }, [gps]);

    useEffect((): void => {
        if (odomEular) {
            const poseOrientation: any | undefined = odomEular.pose?.orientation;
            setCurrentOdomEular(parseFloat(poseOrientation?.y.toFixed(2)) | 0.0);
        }

        if (currentOdomEular) {
            const angle: number = 360 - currentOdomEular;

            if (currRobotMarker) {
                updateRobotMakerIcon(currRobotMarker, angle);
            }
        }
    }, [odomEular]);

    useEffect((): void => {
        if (googleMap) {
            if (path) {
                console.info(`state.path : ${JSON.stringify(path)}`);

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

                setDetectionRagnePolygon(addDetectionRangePolygon(googleMap, Array.from(path)));
                setPathPolyLine(addPathPolyline(googleMap, pathMarkerArray, pathInfoWindowarray));

                if (currRobotMarker) {
                    changeMapCenter(googleMap, currRobotMarker!.getPosition()!);
                }
            }
        } else return;
    }, [path]);

    const focusRouteStatus = (node_index: number, status_code: number): void => {
        const pathInfoContainer: HTMLElement | null = document.getElementById("path_info_container");
        if (pathInfoContainer) {
            const pathInfoElements: HTMLCollectionOf<Element> = pathInfoContainer.getElementsByClassName("path_info");
            if (pathInfoElements.length > 0) {
                switch (status_code) {
                    case 0:
                        (pathInfoElements[node_index] as HTMLElement).style.backgroundColor = "lightblue";
                        (pathInfoElements[node_index] as HTMLElement).scrollIntoView({ behavior: "smooth", block: "center" });
                        break;
                    case 1:
                        (pathInfoElements[node_index] as HTMLElement).style.backgroundColor = "white";
                        break;
                    case 7:
                        closePathSelectModal();
                        googleMap?.setZoom(19);
                        break;        
                    default:
                        (pathInfoElements[node_index] as HTMLElement).style.backgroundColor = "white";
                        break;
                }
            }
        }
    }

    useEffect((): void => {
        if (routeStatus) {
            console.info(`currentRouteStatus : ${JSON.stringify(routeStatus)}`);

            if (routeStatus.node_index === 0) {
                changeMapCenter(googleMap, _pathMarkerArray[routeStatus.node_index].getPosition());
            }

            let driving_flag: boolean = false;
            if (routeStatus.is_driving) {
                driving_flag = true;
            } else {
                driving_flag = false;
            }

            let status: string = "";
            switch (routeStatus.status_code) {
                case 0:
                    status = "출발";
                    focusRouteStatus(routeStatus.node_index, routeStatus.status_code);
                    setIsDriving(true);
                    break;
                case 1:
                    status = "경유지 도착";
                    focusRouteStatus(routeStatus.node_index, routeStatus.status_code);
                    break;
                case 2:
                    status = "주행이 완료되었습니다.";
                    alert(`${status}`);
                    setIsDriving(false);
                    break;
                case 3:
                    status = "주행 서버가 구동되지 않았습니다.";
                    alert(`${status}`);
                    break;
                case 4:
                    status = "주행이 이미 진행 중입니다.";
                    alert(`${status}`);
                    break;
                case 5:
                    status = "주행이 취소되었습니다.";
                    alert(`${status}`);
                    break;
                case 6:
                    status = "주행이 거부되었습니다. 목적지를 확인해주세요.";
                    alert(`${status}`);
                    break;
                case 7:
                    focusRouteStatus(routeStatus.node_index, routeStatus.status_code);
                    break;
                default:
                    break;
            }

            const node_info: any = routeStatus.node_info;

            if (node_info) {
                const currRouteStatus: any = {
                    driving_flag: driving_flag,
                    status: status,
                    node_info: node_info!
                };

                setCurrentRouteStatus(currRouteStatus);
            }
        }
    }, [routeStatus]);

    useEffect((): void => {
        if (cmdVel) {
            if (cmdVel.linear) {
                if (cmdVel.linear.x > 0.0) {
                    setCurrentCmdVel(cmdVel);
                }
            }
        }
    }, [cmdVel]);

    useEffect((): void => {
        if (googleMap) {
            const robotMarker: google.maps.Marker = initializeRobotMarker(googleMap);
            setCurrRobotMarker(robotMarker);

            initializeKECDBorderLine(googleMap);
        }
    }, [googleMap]);

    useEffect((): void => {
        if (currentCmdVel) {
            setCmdVelFlag(true);
        }
    }, [currentCmdVel]);

    useEffect((): () => void => {
        let timer: NodeJS.Timer | null = null;
        if (cmdVelFlag === true) {
            timer = setInterval(() => {
                if (currentCmdVel) {
                    if (currentGps) {
                        const recordedPathCircle: google.maps.Circle = recordNavigatedPathCircle(googleMap, currentGps);
                        _pathCircleArray.push(recordedPathCircle);
                    }
                }
            }, 5000);
        }

        return (): void => {
            if (timer) {
                clearInterval(timer);
            }
            timer = null;
        }
    }, [cmdVelFlag]);

    useEffect((): void => {
        if (rqtt) {
            if (rqttC) {
                const pathTopic: string = `${RTM_TOPIC_FORMAT}/route/path`;
                rqtt.subscribe(rqttC, pathTopic);
                rqtt.addSubscriptionCallback(rqttC, pathTopic, pathCallback);

                const routeStatusTopic: string = `${RTM_TOPIC_FORMAT}/route/status`;
                rqtt.subscribe(rqttC, routeStatusTopic);
                rqtt.addSubscriptionCallback(rqttC, routeStatusTopic, routeStatusCallback);

                const gpsTopic: string = `${RTM_TOPIC_FORMAT}/sensor/ublox/fix`;
                rqtt.subscribe(rqttC, gpsTopic);
                rqtt.addSubscriptionCallback(rqttC, gpsTopic, gpsCallback);

                const odomEularTopic: string = `${RTM_TOPIC_FORMAT}/drive/odom/eular`;
                rqtt.subscribe(rqttC, odomEularTopic);
                rqtt.addSubscriptionCallback(rqttC, odomEularTopic, odomEularCallback);

                const cmdVelTopic: string = `${RTM_TOPIC_FORMAT}/cmd_vel`;
                rqtt.subscribe(rqttC, cmdVelTopic);
                rqtt.addSubscriptionCallback(rqttC, cmdVelTopic, cmdVelCallback);
            }
        }
    }, [rqtt, rqttC]);

    useEffect((): () => void => {
        const mapElement: HTMLElement | null = document.getElementById("map");

        if (mapElement) {
            const mapInstance: google.maps.Map = initializeMap(mapElement, kecCoord);

            setGoogleMap(mapInstance);
            setPathInfoContainer(document.getElementById("path_info_container"));
        }
        localStorage.setItem("isEnableToCommandRoute?", "false");

        return ((): void => {
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
                        <button className={"route_btn_request route_btn_mission_select"} onClick={onMissionSelectClick}>
                            <img src={process.env.PUBLIC_URL + "../marker_mission.png"} />
                            <p>임무 선택</p>
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
                            <PathComponent
                                rqtt={rqtt}
                                rqttC={rqttC}
                            />
                        </Wrapper>
                    </div>
                </ReactModal>
                <ReactModal
                    id="route_path_select_modal"
                    className={"route_path_select_modal"}
                    isOpen={isMissionSelectModalOpen}
                    onRequestClose={closeMissionSelectMoal}
                >
                    <div className="route_path_select_modal_container">
                        <div className="route_path_select_title">
                            <img src={process.env.PUBLIC_URL + "../marker_mission.png"} />
                            <p>임무 선택</p>
                        </div>
                        <div className="route_path_select_close_btn">
                            <img src={process.env.PUBLIC_URL + "../btn_close.png"} onClick={() => { closeMissionSelectMoal() }}></img>
                        </div>
                        <div id="task_list_container" className="task_list_container">
                            <div className="mission_list_container">
                                <p className="mission_list_title">임무 목록</p>
                                <ul id="task_list" className="mission_list">
                                    <li id="task_item" className={"mission_item"} onClick={() => { onTaskClick(missionDeliveryJSON); }}>1. 배송 임무 할당</li>
                                    <li id="task_item" className={"mission_item"} onClick={() => { onTaskClick(missionReturningJSON); }}>2. 복귀 임무 할당</li>
                                </ul>
                            </div>
                            <div className="control_list_container">
                                <p className="control_list_title">제어 목록</p>
                                <ul id="task_list" className="control_list">
                                    <li id="task_item" className={"control_item"} onClick={() => { onTaskClick(controlMoveToDestJSON); }}>1. 하차지 이동 제어</li>
                                    <li id="task_item" className={"control_item"} onClick={() => { onTaskClick(controlMsCompleteReturnJSON); }}>2. 대기 장소 복귀 제어</li>
                                    <li id="task_item" className={"control_item"} onClick={() => { onTaskClick(controlMsCompleteNoReturnJSON); }}>3. 대기 장소 미복귀 제어</li>
                                    <li id="task_item" className={"control_item"} onClick={() => { onTaskClick(controlMsCancelJSON); }}>4. 임무 취소 제어</li>
                                    <li id="task_item" className={"control_item"} onClick={() => { onTaskClick(controlMsCancelJSON); }}>5. 그래프 제어</li>
                                </ul>
                            </div>
                        </div>
                    </div>
                </ReactModal>
            </div>
        </div>
    );
}

export default MapComponent;