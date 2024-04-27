import $ from "jquery";
import React, { useEffect, useRef, useState } from "react";
import "../bluespace/BlueSpaceRequestComponent.css";
import "./MapComponents.css";

interface TestMapComponentProps {
    pathData: any;
    gpsData: any;
    center: naver.maps.LatLng;
    odomEularData: any;
    routeStatus: any;
    onEmergencyStopClick: () => void;
    onEmergencyResumeClick: () => void;
    onGoalCancelClick: () => void;
    onInitClick: () => void;
}

const TestMapComponent: React.FC<TestMapComponentProps> = ({
    center,
    pathData,
    gpsData,
    odomEularData,
    routeStatus,
    onEmergencyStopClick,
    onEmergencyResumeClick,
    onGoalCancelClick,
    onInitClick
}: TestMapComponentProps) => {
    const ref = useRef<HTMLDivElement>(null);
    const [googleMap, setGoogleMap] = useState<google.maps.Map | null>(null);
    const [currentOdomEular, setCurrentOdomEular]: [number | undefined, React.Dispatch<React.SetStateAction<number | undefined>>] = useState<number>();
    const [currentRouteStatus, setCurrentRouteStatus]: [any, React.Dispatch<any>] = useState<any | null>(null);
    const [currMarker, setCurrMarker]: [google.maps.Marker | null, React.Dispatch<React.SetStateAction<google.maps.Marker | null>>] = useState<google.maps.Marker | null>(null);
    const [currentGps, setCurrentGps]: [any, React.Dispatch<any>] = useState<any>({
        status: 0,
        service: 0,
        latitude: 0.0,
        longitude: 0.0
    });

    let pathMarkerArray: Array<google.maps.Marker> = [];
    let pathInfoWindowarray: Array<google.maps.InfoWindow> = [];

    const [currentMode, setCurrentMode]: [string, React.Dispatch<React.SetStateAction<string>>] = useState<string>("KEC");
    const [defaultZoom, setDefaultZoom]: [number, React.Dispatch<React.SetStateAction<number>>] = useState<number>(20);

    const wkValveCoord: naver.maps.LatLng = new naver.maps.LatLng(35.157851, 128.858111);
    const blueSpaceCoord: google.maps.LatLng = new google.maps.LatLng(37.305985, 127.2401652);
    const kecCoord: naver.maps.LatLng = new naver.maps.LatLng(36.1137155, 128.3676005);

    const mapContainer: HTMLDivElement = document.createElement('div');

    const initializeMap: Function = (): void => {
        const instance = new google.maps.Map(ref.current!, {
            center: blueSpaceCoord,
            zoom: 20,
            mapId: '92cb7201b7d43b21',
            disableDefaultUI: true,
            clickableIcons: false,
            minZoom: 10,
            maxZoom: 22,
            gestureHandling: 'greedy',
            restriction: {
                latLngBounds: {
                    north: 39,
                    south: 32,
                    east: 132,
                    west: 124,
                },
                strictBounds: true
            },
        });

        setGoogleMap(instance);
    }

    const initializeRobotMarker: Function = (): void => {
        let iconSymbol = {
            path: google.maps.SymbolPath.FORWARD_CLOSED_ARROW,
            fillColor: 'blue',
            fillOpacity: 1,
            scale: 7,
            strokeWeight: 1,
            rotation: 0 | currentOdomEular!,
        };

        const initialCurrMarker: google.maps.Marker = new google.maps.Marker({
            position: {
                lat: googleMap!.getCenter()?.lat()!,
                lng: googleMap!.getCenter()?.lng()!
            },
            map: googleMap,
            title: "RobotCurrentPos",
            zIndex: 1000,
            clickable: false,
            icon: iconSymbol,
            draggable: false
        });

        setCurrMarker(initialCurrMarker);
    }

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

        let iconSymbol = {
            path: google.maps.SymbolPath.CIRCLE,
            fillColor: 'red',
            fillOpacity: 1,
            scale: 7,
            strokeWeight: 1
        };

        const marker: google.maps.Marker = new google.maps.Marker({
            position: new google.maps.LatLng({ lat: node.position.latitude, lng: node.position.longitude }),
            map: googleMap,
            title: "RobotCurrentPos",
            zIndex: 1000,
            clickable: false,
            icon: iconSymbol,
            draggable: false
        });

        return marker;
    }

    const getClickHandler: Function = (seq: number): Function => {
        return function (e: any) {
            const marker: google.maps.Marker = pathMarkerArray[seq];
            const infoWindow: google.maps.InfoWindow = pathInfoWindowarray[seq];

            infoWindow.open(googleMap);
        }
    }

    const drawPathMarker: Function = (): void => {
        if (pathData) {
            const nodeList: Array<any> = pathData.node_list;

            const uniqueNodeIds: Set<string> = new Set<string>();
            for (const node of nodeList) {
                uniqueNodeIds.add(node.start_node.node_id);
                uniqueNodeIds.add(node.end_node.node_id);
            }

            for (const nodeId of uniqueNodeIds) {
                const node: any = nodeList.find((node) => node.start_node.node_id === nodeId || node.end_node.node_id === nodeId);
                if (node) {
                    const isFirstNode = nodeList.indexOf(node) === 0;
                    const isLastNode = nodeList.indexOf(node) === nodeList.length - 1;

                    if (isFirstNode) {
                        pathMarkerArray.push(addPathMarker(node.start_node, true, false));
                        pathMarkerArray.push(addPathMarker(node.end_node, false, false));
                    } else if (isLastNode) {
                        pathMarkerArray.push(addPathMarker(node.start_node, false, false));
                        pathMarkerArray.push(addPathMarker(node.end_node, false, true));
                    } else {
                        pathMarkerArray.push(addPathMarker(node.start_node, false, false));
                        pathMarkerArray.push(addPathMarker(node.end_node, false, false));
                    }
                }
            }
        }
    }

    const drawPathPolyline: Function = (): void => {
        for (var i = 0, ii = pathMarkerArray.length; i < ii; i++) {

        }

        let path: Array<any> = [];
        for (const pathMarker of pathMarkerArray) {
            // const contentString: string = [
            //     '<div class="iw_inner">',
            //     `   <h3>ID : ${pathMarker!.getTitle().split("/")[0]}</h3>`,
            //     `   <p>종류 : ${pathMarker!.getTitle().split("/")[1]}</p>`,
            //     `   <p>진출 각도 : ${pathMarker!.getTitle().split("/")[2]}</p>`,
            //     `   <p>주행 옵션 : ${pathMarker!.getTitle().split("/")[3]}</p>`,
            //     `   <p>주행 방향 : ${pathMarker!.getTitle().split("/")[4]}</p>`,
            //     `   <p>경도 : ${pathMarker!.getPosition().x}</p>`,
            //     `   <p>위도 : ${pathMarker!.getPosition().y}</p>`,
            //     '</div>'
            // ].join('');

            // const infoWindow: google.maps.InfoWindow = new google.maps.InfoWindow({
            //     content: contentString
            // });

            // pathInfoWindowarray.push(infoWindow);

            // for (var i = 0, ii = pathMarkerArray.length; i < ii; i++) {

            // }
            path.push(pathMarker.getPosition());
        }

        const flightPlanCoordinates = [
            { lat: 37.3061467, lng: 127.2401776 },
            { lat: 37.3061114, lng: 127.2401561 }
        ];

        const polyline: google.maps.Polyline = new google.maps.Polyline({
            map: googleMap,
            path: flightPlanCoordinates,
            geodesic: true,
            strokeColor: "#FF0000",
            strokeOpacity: 1.0,
            strokeWeight: 2
        });
    }

    const changeMapCenter = (coord: google.maps.LatLng): void => {
        googleMap?.setCenter(coord);
    }

    useEffect(() => {
        initializeMap();
    }, []);

    useEffect(() => {
        if (ref.current && googleMap) {
            drawPathMarker();
            drawPathPolyline();
            setDefaultZoom(21);

            if (currMarker) {
                changeMapCenter(currMarker!.getPosition()!);
            }
        } else return;

        return () => {
            setCurrMarker(null);
            setGoogleMap(null);
        }
    }, [googleMap, pathData]);

    useEffect(() => {
        if (gpsData) {
            setCurrentGps({
                status: gpsData.status.status,
                service: gpsData.status.service,
                latitude: parseFloat(gpsData.latitude.toFixed(7)),
                longitude: parseFloat(gpsData.longitude.toFixed(7))
            });

            if (googleMap && currMarker) {
                currMarker!.setPosition({
                    lat: currentGps!.latitude,
                    lng: currentGps!.longitude
                });
            }

            return () => {
                if (currMarker) {
                    currMarker!.setMap(null);
                }
                setCurrMarker(null);
            }
        }
    }, [gpsData]);

    useEffect((): void => {
        if (odomEularData) {
            setCurrentOdomEular(parseFloat(odomEularData.pose.orientation.y.toFixed(2)));
        }

        if (currentOdomEular) {
            const angle: number = 360 - currentOdomEular!;
            $("div[title|='RobotCurrentPos'").css("transform", `rotate(${angle}deg)`);
        }

    }, [odomEularData]);

    useEffect((): void => {
        if (routeStatus) {
            console.info(`currentRouteStatus : ${JSON.stringify(routeStatus)}`);

            let driving_flag: boolean = false;
            if (routeStatus._is_driving) {
                driving_flag = true;
            } else {
                driving_flag = false;
            }

            let status: string = "";
            switch (routeStatus._status_code) {
                case 0:
                    status = "출발";
                    break;
                case 1:
                    status = "경유지 도착";
                    break;
                case 2:
                    status = "주행 완료";
                    break;
                case 5:
                    break;
                default:
                    status = "주행 취소";
                    break;
            }

            const currRouteStatus: any = {
                driving_flag: driving_flag,
                status: status,
                node_info: routeStatus._node_info
            };

            setCurrentRouteStatus(currRouteStatus);
        }
    }, [routeStatus]);

    return (
        <div className={"map_components"}>
            <div className={"map_container"}>
                <div ref={ref} id="map" />
                <div className="data_container">
                    <div className={"gps_data_container"}>
                        <h3>GPS</h3>
                        <div className="">
                            Status : {currentGps.status}
                            <br></br>
                            Service : {currentGps.service}
                            <br></br>
                            경도 : {currentGps.longitude}
                            <br></br>
                            위도 : {currentGps.latitude}
                        </div>
                    </div>
                    <div className={"odom_eular_data_container"}>
                        <h3>차량 각도</h3>
                        <div className="">
                            {currentOdomEular}
                        </div>
                    </div>
                    <div className={"route_status_data_container"}>
                        <h3>주행 상태</h3>
                        <div className="">
                            <div className="route_status_current_route_container">
                                {currentRouteStatus?.is_driving ? `${currentRouteStatus?.node_info[0]} -> ${currentRouteStatus?.node_info[1]}` : ""}
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
}

export default TestMapComponent;