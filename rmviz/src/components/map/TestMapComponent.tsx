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
    const [googleMap, setGoogleMap] = useState<google.maps.Map>();
    const [currentOdomEular, setCurrentOdomEular]: [number | undefined, React.Dispatch<React.SetStateAction<number | undefined>>] = useState<number>();
    const [currentRouteStatus, setCurrentRouteStatus]: [any, React.Dispatch<any>] = useState<any | null>(null);
    const [currMarker, setCurrMarker]: [google.maps.Marker | null, React.Dispatch<React.SetStateAction<google.maps.Marker | null>>] = useState<google.maps.Marker | null>(null);
    const [currentGps, setCurrentGps]: [any, React.Dispatch<any>] = useState<any>({
        status: 0,
        service: 0,
        latitude: 0.0,
        longitude: 0.0
    });

    let pathMarkerArray: Array<naver.maps.Marker> = [];
    let pathInfoWindowarray: Array<naver.maps.InfoWindow> = [];

    const [currentMode, setCurrentMode]: [string, React.Dispatch<React.SetStateAction<string>>] = useState<string>("KEC");
    const [defaultZoom, setDefaultZoom]: [number, React.Dispatch<React.SetStateAction<number>>] = useState<number>(20);

    const wkValveCoord: naver.maps.LatLng = new naver.maps.LatLng(35.157851, 128.858111);
    const blueSpaceCoord: naver.maps.LatLng = new naver.maps.LatLng(37.305985, 127.2401652);
    const kecCoord: naver.maps.LatLng = new naver.maps.LatLng(36.1137155, 128.3676005);

    const mapContainer: HTMLDivElement = document.createElement('div');

    const initializeMap: Function = (): void => {
        const instance = new window.google.maps.Map(mapContainer, {
            center: {
                lat: 37.305985,
                lng: 127.2401652,
            },
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
        const icon = {
            url: process.env.PUBLIC_URL + "marker_current_position.png",
            scaledSize: new google.maps.Size(30, 30),
            origin: new google.maps.Point(0, 0),
            anchor: new google.maps.Point(12, 34)
        };
        const initialCurrMarker: google.maps.Marker = new google.maps.Marker({
            position: {
                lat: currentGps!.latitude,
                lng: currentGps!.longitude,
            },
            map: googleMap,
            title: "RobotCurrentPos",
            zIndex: 1000,
            clickable: false,
            icon: icon
        });

        setCurrMarker(initialCurrMarker);
    }

    useEffect(() => {
        const mapComponents: HTMLElement | null = document.getElementById("map_container");

        mapContainer.id = 'map';
        if (mapComponents) {
            if (mapComponents.firstChild) {
                mapComponents.insertBefore(mapContainer, mapComponents.firstChild);
            } else {
                mapComponents.appendChild(mapContainer);
            }
        }
        initializeMap();
        return () => {
            mapComponents!.removeChild(mapContainer);
        }
    }, []);

    useEffect(() => {
        if (gpsData) {
            setCurrentGps({
                status: gpsData.status.status,
                service: gpsData.status.service,
                latitude: parseFloat(gpsData.latitude.toFixed(7)),
                longitude: parseFloat(gpsData.longitude.toFixed(7))
            });

            initializeRobotMarker();

            return () => {
                setCurrMarker(null);
            }

            // currMarker!.setPosition(new google.maps.LatLng(gpsData.latitude, gpsData.longitude));
            // const markerContainer = document.createElement('div');
            // createRoot(markerContainer).render(<div style={{ backgroundColor: 'yellow', padding: '10px' }}>마커</div>);
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
            <div id="map_container" className={"map_container"}>
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