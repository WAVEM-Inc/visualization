import { useEffect, useState } from "react";
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
    const [isEnableToCommandRoute, setIsEnableToCommandRoute] = useState<string | null>(null);
    const [spathMarkerArray, setPathMarkerArray] = useState<Array<google.maps.Marker>>([]);
    const [pathPolyLine, setPathPolyLine] = useState<google.maps.Polyline | null>(null);
    const [detectionRagnePolygon, setDetectionRagnePolygon] = useState<Array<google.maps.Polygon>>([]);
    const [progress, setProgress] = useState<number>(0);
    const [currentMapFile, setCurrentMapFile] = useState<string>("");

    const requestTopicFormat: string = "/rmviz/request";
    const requestRouteToPoseTopic: string = `${requestTopicFormat}/route_to_pose`;

    let pathMarkerArray: Array<google.maps.Marker> = [];
    let pathInfoWindowarray: Array<google.maps.InfoWindow> = [];

    const buildPathJSON: Function = (path: any): any => {
        const pathJSON: any = {
            isEnableToCommandRoute: isEnableToCommandRoute,
            path: path
        }

        return pathJSON;
    }

    const commandPathClick = (pathJSON: any): void => {
        onClickMqttPublish(mqttClient!, requestRouteToPoseTopic, buildPathJSON(pathJSON));
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
        setIsEnableToCommandRoute(localStorage.getItem("isEnableToCommandRoute?"));
    }, [localStorage.getItem("isEnableToCommandRoute?")]);

    useEffect(() => {
        if (state.path) {
            console.info(`Path File : ${JSON.stringify(state.path)}`);
            if (state.path.paths) {
                setCurrentMapFile(state.path.current_map_file);
                const pathList: Array<any> = Array.from(state.path.paths.path);
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
                            path_id: path.id,
                            path_name: path.name
                        };

                        li.onclick = () => commandPathClick(pathJSON);
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
                state.path = null;
            }
        });
    }, []);

    const handleProgressChange = (e: React.ChangeEvent<HTMLInputElement>) => {
        const newProgress: number = Number(e.target.value);
        setProgress(newProgress);
        changeMapCenter(googleMap, spathMarkerArray[newProgress].getPosition()!);
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
            <div className="path_list_info_container">
                {spathMarkerArray.length > 0 && (
                    <div className="progress_container">
                        <input
                            type="range"
                            min="0"
                            max={spathMarkerArray.length - 1}
                            value={progress}
                            onChange={handleProgressChange}
                            className="progress_slider"
                        />
                        <div className="progress_info">
                            <p>Current Node: {progress + 1}</p>
                            <p>ID: {spathMarkerArray[progress].getTitle()!.split("/")[0]}</p>
                        </div>
                    </div>
                )}
            </div>
        </div>
    );
}

export default GoogleMapPathComponent;