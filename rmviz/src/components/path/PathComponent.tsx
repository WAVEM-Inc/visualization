import axios, { AxiosResponse } from "axios";
import mqtt from "mqtt/*";
import React, { useEffect, useRef, useState } from "react";
import { addDetectionRangePolygon, addPathMarker, addPathPolyline, changeMapCenter, initializeMap } from "../../service/map/MapService";
import { onClickMqttPublish } from "../../utils/Utils";
import "./PathComponent.css";
import Rqtt from "../../api/application/rqtt";
import { RTM_TOPIC_FORMAT } from "../../api/domain/common.constants";

interface PathComponentProps {
    rqtt: Rqtt,
    rqttC: mqtt.MqttClient;
}

const PathComponent: React.FC<PathComponentProps> = ({
    rqtt,
    rqttC
}: PathComponentProps) => {
    const [path, setPath] = useState<any | null>(null);

    const [pathResponse, setPathResponse] = useState<any | null>(null);
    const [googleMap, setGoogleMap] = useState<google.maps.Map>();
    const kecCoord: google.maps.LatLng = new google.maps.LatLng(36.11434, 128.3690);
    const [_pathMarkerArray, setPathMarkerArray] = useState<Array<google.maps.Marker>>([]);
    const [pathPolyLine, setPathPolyLine] = useState<google.maps.Polyline | null>(null);
    const [detectionRagnePolygon, setDetectionRagnePolygon] = useState<Array<google.maps.Polygon>>([]);
    const [pathProgress, setPathProgress] = useState<number>(0);
    const [currentMapFile, setCurrentMapFile] = useState<string>("");
    const [pathList, setPathList] = useState<Array<any>>([]);
    const [pathListIndex, setPathListIndex] = useState<number>(0);
    const [selectedPathListIndex, setSelectedPathListIndex] = useState<number | null>(null);
    const [editedPath, setEditedPath] = useState<any>({});
    const [pathSource, setPathSource] = useState<any>({});
    const [pathGoal, setPathGoal] = useState<any>({});
    const [pathGoalIndex, setPathGoalIndex] = useState<number>(0);

    const pathProgressImageRef = useRef<HTMLImageElement | null>(null);

    const requestTopicFormat: string = "net/wavem/rms/rqtt/mtr";
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

    const requestPathView: Function = (pathJSON: any): void => {
        const pathJSONBuilt: any = buildPathJSON(pathJSON, false);
        onClickMqttPublish(rqtt, rqttC, requestRouteToPoseTopic, pathJSONBuilt);
    }

    const requestPathCommand: Function = (pathJSON: any): void => {
        const pathJSONBuilt: any = buildPathJSON(pathJSON, true);
        onClickMqttPublish(rqtt, rqttC, requestRouteToPoseTopic, pathJSONBuilt);
    }

    const requestRenewPath: React.MouseEventHandler<HTMLButtonElement> = (): void => {
        onClickMqttPublish(rqtt, rqttC, requestPathRenewTopic, {});
    }

    const changePathSource: React.MouseEventHandler<HTMLButtonElement> = (): void => {
        console.info(`pL : ${JSON.stringify(pathList[pathListIndex])}`);
        const nodeList: Array<any> = pathList[pathListIndex].nodeList

        if (pathProgress < nodeList.length - 1) {
            setPathProgress(pathProgress + 1);
            const updatedPathList: any = nodeList.filter((it, idx) => idx > pathProgress);

            for (const p of updatedPathList) {
                console.log(`p : ${JSON.stringify(p)}`);
            }

            const pathJSON: any = {
                id: pathList[pathListIndex].id,
                name: pathList[pathListIndex].name,
                nodeList: updatedPathList
            };
            setEditedPath(pathJSON);
        } else {
            alert("출발지와 도착지가 동일하거나 출발지가 도착지를 넘어섭니다.");
        }
    }

    const changePathGoal: React.MouseEventHandler<HTMLButtonElement> = (): void => {
        if (pathListIndex > 0) {
            setPathListIndex(pathListIndex - 1);
            const updatedPathList = pathList.slice();
            updatedPathList.push(updatedPathList.splice(pathListIndex, 1)[0]);
            const pathJSON = {
                id: updatedPathList[updatedPathList.length - 1].id,
                name: updatedPathList[updatedPathList.length - 1].name
            };
            setEditedPath(pathJSON);
        } else {
            alert("출발지와 도착지가 동일하거나 도착지가 출발지보다 앞에 위치합니다.");
        }
    }

    const drawPathMarker: Function = (): void => {
        if (path || pathResponse) {
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
                        setPathSource(node);
                    } else if (isLastNode) {
                        pathMarkerArray.push(addPathMarker(googleMap, node, false, true));
                        setPathGoal(node);
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
        _pathMarkerArray.forEach(marker => marker.setMap(null));
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

    const handleProgressChange = (index: number) => {
        setPathProgress(index);
        changeMapCenter(googleMap, _pathMarkerArray[index].getPosition()!);
    }

    const httpLoadPathRequest: Function = async (): Promise<void> => {
        try {
            const response: AxiosResponse<any, any> = await axios.post("/v1/api/path/select");
            console.info(`Path HTTP : ${JSON.stringify(response)}`);
            setPathResponse(response.data);
        } catch (error) {
            console.error("Error fetching Path data", error);
        }
    }

    const pathCallback: (message: any) => void = (message: any): void => {
        setPath(message);
    }

    useEffect(() => {
        if (pathResponse) {
            if (pathResponse.paths) {
                setCurrentMapFile(pathResponse.current_map_file);
                const pathList: Array<any> = Array.from(pathResponse.paths.path);
                setPathList(pathList);
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
                            setPathListIndex(index);
                            requestPathView(pathJSON);
                            setSelectedPathListIndex(index);
                        };

                        pathListElement.appendChild(li);
                    });
                }
            }

            if (path) {
                flushPath();
                drawPathMarker();

                setDetectionRagnePolygon(addDetectionRangePolygon(googleMap, Array.from(path)));
                setPathPolyLine(addPathPolyline(googleMap, pathMarkerArray, pathInfoWindowarray));
            }
        }
    }, [pathResponse, path]);

    useEffect(() => {
        if (pathListIndex) {
            setEditedPath(pathList[pathListIndex]);
        }
    }, [pathListIndex]);

    useEffect(() => {
        if (selectedPathListIndex !== null) {
            const pathListElement = document.getElementById("path_list");
            if (pathListElement) {
                Array.from(pathListElement.children).forEach((child, index) => {
                    (child as HTMLElement).style.backgroundColor = index === selectedPathListIndex ? "lightblue" : "";
                });
            }
        }
    }, [pathList, selectedPathListIndex]);


    useEffect(() => {
        const pathProgressMarkerInfoElements: NodeListOf<HTMLElement> | null = document.querySelectorAll(".path_progress_marker_info");

        if (pathProgressMarkerInfoElements && _pathMarkerArray) {
            pathProgressMarkerInfoElements.forEach((markerInfo) => {
                const imgElements: HTMLCollectionOf<HTMLImageElement> = markerInfo.getElementsByTagName("img");
                for (let i = 0; i < imgElements.length; i++) {
                    imgElements[i].remove();
                }
            });

            pathProgressMarkerInfoElements.forEach((markerInfo, index) => {
                const pathProgressMarkerIconImg: HTMLImageElement = document.createElement("img");
                const markerType: string | undefined = _pathMarkerArray[index]?.getTitle()?.split("/")[1];

                switch (markerType) {
                    case "intersection":
                        pathProgressMarkerIconImg.src = process.env.PUBLIC_URL + "../marker_intersection.png";
                        break;
                    case "endpoint":
                        if (index === 0) {
                            pathProgressMarkerIconImg.src = process.env.PUBLIC_URL + "../marker_start.png";
                        } else if (index !== pathProgressMarkerInfoElements.length - 1) {
                            pathProgressMarkerIconImg.src = process.env.PUBLIC_URL + "../marker_landmark.png";
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

            setPathProgress(0);
            googleMap?.setZoom(20);

            if (_pathMarkerArray[0]) {
                changeMapCenter(googleMap, _pathMarkerArray[0].getPosition());
            }

            console.info(`_pathMarkerArray length : ${_pathMarkerArray.length}`);
            setPathGoalIndex(_pathMarkerArray.length);
        }
    }, [_pathMarkerArray]);

    useEffect(() => {
        if (pathSource) {
            console.info(`pathSource : ${pathSource.nodeId}`);
        }

        if (pathGoal) {
            console.info(`pathGoal : ${pathGoal.nodeId}`);
        }
    }, [pathSource, pathGoal]);

    useEffect(() => {
        if (pathProgress) {
            console.info(`pathProgress : ${pathProgress}, length : ${_pathMarkerArray.length}`);

            if (pathProgress === pathGoalIndex - 1) {
                alert("출발지와 목적지가 동일합니다.");
                setPathProgress(pathProgress - 1);
            } else {
                const nodeList: Array<any> = Array.from(path);
                setPathSource(nodeList[pathProgress]);
                const pathProgressMarkerInfoElements: NodeListOf<HTMLElement> | null = document.querySelectorAll(".path_progress_marker_info");

                if (pathProgressMarkerInfoElements) {
                    const _imgElements: HTMLCollectionOf<HTMLImageElement> = pathProgressMarkerInfoElements[pathProgress - 1].getElementsByTagName("img");
                    _imgElements[0].src = process.env.PUBLIC_URL + "../marker_landmark.png";

                    const imgElements: HTMLCollectionOf<HTMLImageElement> = pathProgressMarkerInfoElements[pathProgress].getElementsByTagName("img");
                    imgElements[0].src = process.env.PUBLIC_URL + "../marker_start.png";
                }
            }
        }
    }, [pathProgress]);

    useEffect(() => {
        if (pathGoalIndex) {
            console.info(`pathGoalIndex : ${pathGoalIndex}, pathProgress : ${pathProgress}`);

            if (pathGoalIndex === pathProgress) {
                alert("목적지와 출발지가 동일합니다.");
                setPathGoalIndex(pathGoalIndex + 1);
            } else {
                const nodeList: Array<any> = Array.from(path);
                setPathGoal(nodeList[pathGoalIndex]);
            }
        }
    }, [pathGoalIndex]);

    useEffect(() => {
        if (rqtt) {
            if (rqttC) {
                const pathTopic: string = `${RTM_TOPIC_FORMAT}/route/path`;
                rqtt.subscribe(rqttC, pathTopic);
                rqtt.addSubscriptionCallback(rqttC, pathTopic, pathCallback);
            }
        }
    }, [rqtt, rqttC]);

    useEffect(() => {
        const mapElement: HTMLElement | null = document.getElementById("path_map");

        if (mapElement) {
            const mapInstance: google.maps.Map = initializeMap(mapElement, kecCoord);

            setGoogleMap(mapInstance);
        }

        httpLoadPathRequest();

        return (() => {
            if (googleMap) {
                googleMap.unbindAll();
            }
            if (pathResponse) {
                flushPath();
                setPathResponse(null);
                setSelectedPathListIndex(null);
            }
        });
    }, []);

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
                {_pathMarkerArray.length > 0 && (
                    <div className="path_controller_cotainer">
                        <div className="path_progress_container">
                            <div className="path_progress_slider">
                                {_pathMarkerArray.map((marker, index) => (
                                    <React.Fragment key={index}>
                                        <div
                                            className={`path_progress_cylinder ${index === pathProgress ? 'active' : ''}`}
                                            onClick={() => handleProgressChange(index)}
                                            style={{ left: `${(index / (_pathMarkerArray.length - 1)) * 100}%` }}
                                        >
                                            <div id="path_progress_marker_info" className="path_progress_marker_info">
                                                <p>{marker.getTitle()?.split("/")[0]}</p>
                                            </div>
                                            <div className="path_progress_cylinder_bar"></div>
                                        </div>
                                        {index < _pathMarkerArray.length - 1 && (
                                            <div
                                                className="path_progress_horizontal_bar"
                                                style={{
                                                    left: `${((index) / (_pathMarkerArray.length - 1)) * 100}%`,
                                                    width: `${100 / (_pathMarkerArray.length - 1)}%`,
                                                    bottom: '8px'
                                                }}
                                            ></div>
                                        )}
                                    </React.Fragment>
                                ))}
                            </div>
                        </div>
                        <div className="path_command_btn_container">
                            <div className="path_command_btn_grid">
                                <button className="path_command_btn source_change_btn" onClick={changePathSource}>
                                    <img src={process.env.PUBLIC_URL + "../right_arrow.png"} />
                                    <p>출발지 변경</p>
                                </button>
                                <button className="path_command_btn goal_change_btn" onClick={changePathGoal}>
                                    <img src={process.env.PUBLIC_URL + "../right_arrow.png"} />
                                    <p>도착지 변경</p>
                                </button>
                                <button className="path_command_btn renew_path_btn" onClick={requestRenewPath}>
                                    <img src={process.env.PUBLIC_URL + "../marker_refresh.png"} />
                                    <p>경로 갱신</p>
                                </button>
                                <button className="path_command_btn command_path_btn" onClick={() => {
                                    requestPathCommand(pathList[pathListIndex]);
                                 }}>
                                    <img src={process.env.PUBLIC_URL + "../go_sign.png"}></img>
                                    <p>주행 명령</p>
                                </button>
                            </div>
                        </div>
                    </div>
                )}
            </div>
        </div>
    );
}

export default PathComponent;