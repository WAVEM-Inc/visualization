import { faLocationArrow } from "@fortawesome/free-solid-svg-icons";
import React, { useEffect, useState } from "react";
import MqttClient from "../../api/mqttClient";
import * as emergencyResumeJSON from "../../assets/json/common/emergency_resume.json";
import * as emergencyStopJSON from "../../assets/json/common/emergency_stop.json";
import { MapState } from "../../domain/map/MapDomain";
import { onClickMqttPublish } from "../../utils/Utils";
import "./GoogleMapComponent.css";

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

        const marker: google.maps.Marker = new google.maps.Marker({
            position: new google.maps.LatLng(node.position.latitude, node.position.longitude),
            map: googleMap,
            title: nodeTitle,
            icon: {
                url: iconUrl,
                size: new google.maps.Size(30, 30),
                scaledSize: new google.maps.Size(30, 30),
                origin: new google.maps.Point(0, 0),
                anchor: new google.maps.Point(12, 34)
            }
        });

        return marker;
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

    const getClickHandler: Function = (seq: number): Function => {
        let isOpened: number = 0;
        return function (e: any) {
            const marker: google.maps.Marker = pathMarkerArray[seq];
            const infoWindow: google.maps.InfoWindow = pathInfoWindowarray[seq];

            if (isOpened == 0) {
                infoWindow.open(googleMap!, marker);
                isOpened++;
            } else {
                infoWindow.close();
                isOpened = 0;
            }
        }
    }

    const changeMapCenter = (coord: google.maps.LatLng): void => {
        googleMap?.setCenter(coord);
    }

    const drawPathPolyline: Function = (): void => {
        for (var i = 0, ii = pathMarkerArray.length; i < ii; i++) {
            google.maps.event.addListener(pathMarkerArray[i], "click", getClickHandler(i));
        }

        let path: Array<any> = [];
        for (const pathMarker of pathMarkerArray) {
            const contentString: string = [
                '<div class="node_info_window">',
                `   <h3>ID : ${pathMarker!.getTitle()!.split("/")[0]}</h3>`,
                `   <p>종류 : ${pathMarker!.getTitle()!.split("/")[1]}</p>`,
                `   <p>진출 각도 : ${pathMarker!.getTitle()!.split("/")[2]}</p>`,
                `   <p>주행 옵션 : ${pathMarker!.getTitle()!.split("/")[3]}</p>`,
                `   <p>주행 방향 : ${pathMarker!.getTitle()!.split("/")[4]}</p>`,
                `   <p>경도 : ${pathMarker!.getPosition()!.lng()}</p>`,
                `   <p>위도 : ${pathMarker!.getPosition()!.lat()}</p>`,
                '</div>'
            ].join('');

            const infoWindow: google.maps.InfoWindow = new google.maps.InfoWindow({
                content: contentString
            });

            pathInfoWindowarray.push(infoWindow);

            for (var i = 0, ii = pathMarkerArray.length; i < ii; i++) {
                google.maps.event.addListener(pathMarkerArray[i], "click", getClickHandler(i));
            }
            path.push(pathMarker.getPosition());
        }

        const polyline: google.maps.Polyline = new google.maps.Polyline({
            map: googleMap,
            path: path,
            clickable: true,
            strokeColor: "red",
            strokeOpacity: 1.0,
            strokeWeight: 5.0
        });
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
                    currRobotMarker!.setPosition(new google.maps.LatLng(state.gps.latitude, state.gps.longitude));
                } else return;
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
                const iconOpts: any = {
                    path: faLocationArrow.icon[4] as string,
                    fillColor: "#0000ff",
                    fillOpacity: 1,
                    anchor: new google.maps.Point(
                        faLocationArrow.icon[0] / 2,
                        faLocationArrow.icon[1]
                    ),
                    strokeWeight: 2,
                    strokeColor: "black",
                    scale: 0.065,
                    rotation: -45.0 + angle
                }
                currRobotMarker.setIcon(iconOpts);
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

                drawPathMarker();
                drawPathPolyline();

                if (currRobotMarker) {
                    changeMapCenter(currRobotMarker!.getPosition()!);
                }
            }
        } else return;
    }, [state.path]);

    useEffect(() => {
        if (googleMap) {
            const robotMarker: google.maps.Marker = new google.maps.Marker({
                position: googleMap.getCenter(),
                map: googleMap!,
                title: "RobotCurrentPos",
                zIndex: 1000,
                clickable: false,
                icon: {
                    path: faLocationArrow.icon[4] as string,
                    fillColor: "#0000ff",
                    fillOpacity: 1,
                    anchor: new google.maps.Point(
                        faLocationArrow.icon[0] / 2,
                        faLocationArrow.icon[1]
                    ),
                    strokeWeight: 2,
                    strokeColor: "black",
                    scale: 0.065,
                    rotation: -45.0
                }
            });
            setCurrRobotMarker(robotMarker);
        }
    }, [googleMap]);

    useEffect(() => {
        const mapElement = document.getElementById("map");

        if (mapElement) {
            const mapInstance = new google.maps.Map(mapElement, {
                center: kecCoord,
                zoom: 18,
                minZoom: 16,
                maxZoom: 21,
                restriction: {
                    latLngBounds: {
                        north: 39,
                        south: 32,
                        east: 132,
                        west: 124,
                    },
                    strictBounds: true
                },
                mapTypeControl: true,
                mapTypeId: google.maps.MapTypeId.SATELLITE
            });

            setGoogleMap(mapInstance);

            return (() => {

            });
        }
    }, []);

    useEffect(() => {
        setPathInfoContainer(document.getElementById("path_info_container"));
    }, []);

    return (
        <div className={"map_components"}>
            <div className={"map_container"}>
                <div id="map"></div>
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
}

export default GoogleMapComponent;