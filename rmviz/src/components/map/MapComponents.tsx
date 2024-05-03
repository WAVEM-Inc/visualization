import $ from "jquery";
import React, { useEffect, useRef, useState } from "react";
import { MapState } from "../../domain/map/MapDomain";
import { initializeMap, initializeRobotMarker } from "../../service/MapService";
import "./MapComponents.css";

interface MapComponentProps {
    center: naver.maps.LatLng;
    state: MapState;
    onEmergencyStopClick: () => void;
    onEmergencyResumeClick: () => void;
    onGoalCancelClick: () => void;
    onInitClick: () => void;
}

const MapComponent: React.FC<MapComponentProps> = ({
    state,
    center,
    onEmergencyStopClick,
    onEmergencyResumeClick,
    onGoalCancelClick,
    onInitClick
}: MapComponentProps): React.ReactElement<any, any> | null => {
    const { naver }: Window & typeof globalThis = window;
    const mapRef: React.MutableRefObject<HTMLDivElement | null> = useRef<HTMLDivElement>(null);

    let map: naver.maps.Map | null = null;
    const [currRobotMarker, setCurrRobotMarker]: [naver.maps.Marker | null, React.Dispatch<React.SetStateAction<naver.maps.Marker | null>>] = useState<naver.maps.Marker | null>(null);
    const [currRobotFilteredMarker, setCurrRobotFilteredMarker]: [naver.maps.Marker | null, React.Dispatch<React.SetStateAction<naver.maps.Marker | null>>] = useState<naver.maps.Marker | null>(null);
    const [currentGps, setCurrentGps]: [any, React.Dispatch<any>] = useState<any>({
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

    const [currentMode, setCurrentMode]: [string, React.Dispatch<React.SetStateAction<string>>] = useState<string>("KEC");
    const [defaultZoom, setDefaultZoom]: [number, React.Dispatch<React.SetStateAction<number>>] = useState<number>(20);

    const wkValveCoord: naver.maps.LatLng = new naver.maps.LatLng(35.157851, 128.858111);
    const blueSpaceCoord: naver.maps.LatLng = new naver.maps.LatLng(37.305985, 127.2401652);
    const kecCoord: naver.maps.LatLng = new naver.maps.LatLng(36.1137155, 128.3676005);

    const addPathMarker: Function = (node: any, is_start: boolean, is_end: boolean): any => {
        console.info(`addPathMarker node : ${JSON.stringify(node)}`);

        let iconUrl: string = "";

        if (is_start && !is_end) {
            iconUrl = process.env.PUBLIC_URL + "marker_start.png";
        } else if (!is_start && is_end) {
            iconUrl = process.env.PUBLIC_URL + "marker_arrive.png";
        } else {
            iconUrl = process.env.PUBLIC_URL + "marker_landmark.png";
        }

        const marker: naver.maps.Marker = new naver.maps.Marker({
            position: new naver.maps.LatLng(node.position.latitude, node.position.longitude),
            map: map,
            title: `${node.nodeId.split("-")[2]}/${node.kind}/${node.heading}/${node.drivingOption}/${node.direction}`,
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
                    const isFirstNode = nodeList.indexOf(node) === 0;
                    const isLastNode = nodeList.indexOf(node) === nodeList.length - 1;

                    if (isFirstNode) {
                        pathMarkerArray.push(addPathMarker(node, true, false));
                    } else if (isLastNode) {
                        pathMarkerArray.push(addPathMarker(node, false, true));
                    } else {
                        pathMarkerArray.push(addPathMarker(node, false, false));
                    }
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

    useEffect((): void => {
        if (mapRef.current && naver && !map) {
            map = initializeMap(mapRef.current, center);
        } else return;

        if (mapRef.current && naver && map) {
            const robotMarker: naver.maps.Marker = initializeRobotMarker(map);
            setCurrRobotMarker(robotMarker);
        }

        if (mapRef.current && naver && map) {
            if (state.path) {
                console.info(`state.path : ${JSON.stringify(state.path)}`);
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

    useEffect(() => {
        return () => {
            if (map) {
                setCurrRobotMarker(null);
                map.destroy();
                map = null;
            }
        }
    }, []);

    return (
        <div className={"map_components"}>
            <div className={"map_container"}>
                <div ref={mapRef} id={"map"} />
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
                    <div className={"route_status_data_container"}>
                        <div className="data_title">
                            주행 상태
                        </div>
                        <div className="data_data">
                            <div className="route_status_current_route_container">
                                {"["}{currentRouteStatus?.node_info[0]}{"]"} {"->"} {"["}{currentRouteStatus?.node_info[1]}{"]"}
                            </div>
                            <div className="">
                                주행 중 : {currentRouteStatus?.driving_flag.toString()}
                            </div>
                            <div className="">
                                상태 : {currentRouteStatus?.status}
                            </div>
                        </div>
                    </div>
                    <div className="route_request_btn_container">
                        <button className={"route_btn_request route_btn_can_init"} onClick={onInitClick}>CAN 초기화</button>
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