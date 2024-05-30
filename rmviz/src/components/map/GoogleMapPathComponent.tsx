import React, { useEffect, useState } from "react";
import MqttClient from "../../api/mqttClient";
import { MapState } from "../../domain/map/MapDomain";
import { addDetectionRangePolygon, addPathMarker, addPathPolyline, changeMapCenter, initializeMap } from "../../service/map/MapService";
import { onClickMqttPublish } from "../../utils/Utils";
import "./GoogleMapPathComponent.css";

interface GoogleMapPathComponentProps {
    mqttClient: MqttClient;
    state: MapState;
}

const GoogleMapPathComponent: React.FC<GoogleMapPathComponentProps> = ({
    mqttClient,
    state
}: GoogleMapPathComponentProps) => {
    const [googleMap, setGoogleMap] = useState<google.maps.Map>();
    const kecCoord: google.maps.LatLng = new google.maps.LatLng(36.11434, 128.3690);
    const [spathMarkerArray, setPathMarkerArray] = useState<Array<google.maps.Marker>>([]);
    const [pathPolyLine, setPathPolyLine] = useState<google.maps.Polyline | null>(null);
    const [detectionRagnePolygon, setDetectionRagnePolygon] = useState<Array<google.maps.Polygon>>([]);
    const [progress, setProgress] = useState<number>(0);
    const [currentMapFile, setCurrentMapFile] = useState<string>("");
    const [paths, setPaths] = useState<Array<any>>([]);
    const [routeIndex, setRouteIndex] = useState<number>(0);

    const requestTopicFormat: string = "/rmviz/request";
    const requestRouteToPoseTopic: string = `${requestTopicFormat}/route_to_pose`;
    const requestPathRenewTopic: string = `${requestTopicFormat}/path/renew`;

    let pathMarkerArray: Array<google.maps.Marker> = [];
    let pathInfoWindowarray: Array<google.maps.InfoWindow> = [];

    const buildPathJSON: Function = (path: any, isEnableToCommandRoute: boolean): any => {
        const pathJSON: any = {
            isEnableToCommandRoute: isEnableToCommandRoute,
            path: path
        }

        return pathJSON;
    }

    const showPathClick = (pathJSON: any): void => {
        const pathJSONBuilt: any = buildPathJSON(pathJSON, false);
        onClickMqttPublish(mqttClient!, requestRouteToPoseTopic, pathJSONBuilt);
    }

    const commandPathClick = (pathJSON: any): void => {
        const pathJSONBuilt: any = buildPathJSON(pathJSON, true);
        onClickMqttPublish(mqttClient!, requestRouteToPoseTopic, pathJSONBuilt);
        localStorage.setItem("closePathSelectModal", "true");
    }

    const onPathRenewClick = (): void => {
        onClickMqttPublish(mqttClient!, requestPathRenewTopic, {});
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
    }

    useEffect(() => {
        if (state.path) {
            console.info(`Path File : ${JSON.stringify(state.path)}`);
            if (state.path.paths) {
                setCurrentMapFile(state.path.current_map_file);
                const pathList: Array<any> = Array.from(state.path.paths.path);
                setPaths(pathList);
                const pathListElement: HTMLElement | null = document.getElementById("path_list");

                if (pathListElement) {
                    pathListElement.innerHTML = "";

                    pathList.forEach((path, index) => {
                        const li: HTMLLIElement = document.createElement("li");
                        li.className = "path_list_element";
                        li.innerHTML = [
                            `<div class="path_list_element_index">${index + 1}</div>`,
                            `<div class="path_list_element_name">${path.name}</div>`
                        ].join(``);

                        const pathJSON: any = {
                            id: path.id,
                            name: path.name
                        };

                        li.onclick = () => {
                            setRouteIndex(index);
                            showPathClick(pathJSON);
                        };
                        pathListElement.appendChild(li);
                    });
                }
            }

            if (state.path) {
                console.info(`state.path : ${JSON.stringify(state.path)}`);

                flushPath();
                drawPathMarker();

                setDetectionRagnePolygon(addDetectionRangePolygon(googleMap, Array.from(state.path)));
                setPathPolyLine(addPathPolyline(googleMap, pathMarkerArray, pathInfoWindowarray));
            }
        }
    }, [state.path]);

    useEffect(() => {
        const pathProgressMarkerInfoElements: NodeListOf<HTMLElement> | null = document.querySelectorAll(".path_progress_marker_info");

        if (pathProgressMarkerInfoElements && spathMarkerArray) {
            pathProgressMarkerInfoElements.forEach((markerInfo) => {
                const imgElements: HTMLCollectionOf<HTMLImageElement> = markerInfo.getElementsByTagName("img");
                for (let i = 0; i < imgElements.length; i++) {
                    imgElements[i].remove();
                }
            });

            pathProgressMarkerInfoElements.forEach((markerInfo, index) => {
                const pathProgressMarkerIconImg: HTMLImageElement = document.createElement("img");
                const markerType = spathMarkerArray[index]?.getTitle()?.split("/")[1];

                switch (markerType) {
                    case "intersection":
                        pathProgressMarkerIconImg.src = process.env.PUBLIC_URL + "../marker_intersection.png";
                        break;
                    case "endpoint":
                        if (index === 0) {
                            pathProgressMarkerIconImg.src = process.env.PUBLIC_URL + "../marker_start.png";
                        } else {
                            pathProgressMarkerIconImg.src = process.env.PUBLIC_URL + "../marker_arrive.png";
                        }
                        break;
                    default:
                        if (index === 0) {
                            pathProgressMarkerIconImg.src = process.env.PUBLIC_URL + "../marker_start.png";
                        } else if (index === pathProgressMarkerInfoElements.length - 1) {
                            pathProgressMarkerIconImg.src = process.env.PUBLIC_URL + "../marker_arrive.png";
                        } else {
                            pathProgressMarkerIconImg.src = process.env.PUBLIC_URL + "../marker_landmark.png";
                        }
                        break;
                }

                markerInfo.appendChild(pathProgressMarkerIconImg);
            });
        }
    }, [spathMarkerArray]);

    useEffect(() => {
        const mapElement: HTMLElement | null = document.getElementById("path_map");

        if (mapElement) {
            const mapInstance: google.maps.Map = initializeMap(mapElement, kecCoord);

            setGoogleMap(mapInstance);
        }

        return (() => {
            if (googleMap) {
                googleMap.unbindAll();
            }
            if (state.path) {
                setPathMarkerArray([]);
                pathMarkerArray = [];
                state.path = null;
            }
        });
    }, []);

    const handleProgressChange = (index: number) => {
        setProgress(index);
        changeMapCenter(googleMap, spathMarkerArray[index].getPosition()!);
    }

    return (
        <div className={"path_map_components"}>
            <div className={"path_map_container"}>
                <div id="path_map" />
                <div id="path_list_container" className="path_list_container">
                    <p className="path_list_title">경로 목록({currentMapFile})</p>
                    <ul id="path_list" className="path_list"></ul>
                </div>
            </div>
            <div className="path_util_container">
                {spathMarkerArray.length > 0 && (
                    <div className="path_controller_cotainer">
                        <div className="path_progress_container">
                            <div className="path_progress_slider">
                                {spathMarkerArray.map((marker, index) => (
                                    <React.Fragment key={index}>
                                        <div
                                            className={`path_progress_cylinder ${index === progress ? 'active' : ''}`}
                                            onClick={() => handleProgressChange(index)}
                                            style={{ left: `${(index / (spathMarkerArray.length - 1)) * 100}%` }}
                                        >
                                            <div id="path_progress_marker_info" className="path_progress_marker_info">
                                                <p>{marker.getTitle()?.split("/")[0]}</p>
                                            </div>
                                            <div className="path_progress_cylinder_bar"></div>
                                        </div>
                                        {index === progress && (
                                            <img
                                                src={process.env.PUBLIC_URL + "../marker_current_position.png"}
                                                className="path_progress_image"
                                                style={{
                                                    left: `${(index / (spathMarkerArray.length - 1)) * 100}%`,
                                                    transform: 'translateX(-50%) rotate(90deg)',
                                                }}
                                            />
                                        )}
                                        {index < spathMarkerArray.length - 1 && (
                                            <div
                                                className="path_progress_horizontal_bar"
                                                style={{
                                                    left: `${((index) / (spathMarkerArray.length - 1)) * 100}%`,
                                                    width: `${100 / (spathMarkerArray.length - 1)}%`,
                                                    bottom: '8px'
                                                }}
                                            ></div>
                                        )}
                                    </React.Fragment>
                                ))}
                            </div>
                        </div>
                        <div className="path_command_btn_container">
                            <button className={"path_command_btn renew_path_btn"} onClick={onPathRenewClick}>
                                <img src={process.env.PUBLIC_URL + "../marker_refresh.png"} />
                                <p>경로 갱신</p>
                            </button>
                            <button className="path_command_btn command_path_btn" onClick={() => {
                                commandPathClick(paths[routeIndex]);
                            }}>
                                <img src={process.env.PUBLIC_URL + "../go_sign.png"}></img>
                                <p>주행 명령</p>
                            </button>
                        </div>
                    </div>
                )}
            </div>
        </div>
    );
}

export default GoogleMapPathComponent;