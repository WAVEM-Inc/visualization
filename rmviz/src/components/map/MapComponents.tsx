import $ from "jquery";
import React, { useEffect, useRef, useState } from "react";
import "./MapComponents.css";

interface MapComponentProps {
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

const MapComponent: React.FC<MapComponentProps> = ({
    center,
    pathData,
    gpsData,
    odomEularData,
    routeStatus,
    onEmergencyStopClick,
    onEmergencyResumeClick,
    onGoalCancelClick,
    onInitClick
}: MapComponentProps) => {
    const { naver }: Window & typeof globalThis = window;
    const mapRef: React.MutableRefObject<HTMLDivElement | null> = useRef<HTMLDivElement>(null);

    let map: naver.maps.Map | null = null;
    const [currMarker, setCurrMarker]: [naver.maps.Marker | null, React.Dispatch<React.SetStateAction<naver.maps.Marker | null>>] = useState<naver.maps.Marker | null>(null);
    const [currentGps, setCurrentGps]: [any, React.Dispatch<any>] = useState<any>({
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

    const initializeMap: Function = (): void => {
        const openStreetMapType: naver.maps.ImageMapType = new naver.maps.ImageMapType({
            name: "OSM",
            minZoom: 0,
            maxZoom: defaultZoom,
            tileSize: new naver.maps.Size(256, 256),
            projection: naver.maps.EPSG3857,
            repeatX: true,
            tileSet: [
                "https://a.tile.openstreetmap.org/{z}/{x}/{y}.png",
                "https://b.tile.openstreetmap.org/{z}/{x}/{y}.png",
                "https://c.tile.openstreetmap.org/{z}/{x}/{y}.png"
            ],
            provider: [{
                title: " /OpenStreetMap",
                link: "http://www.openstreetmap.org/copyright"
            }]
        });

        const mapOpts: any = {
            center: center,
            mapTypeId: naver.maps.MapTypeId.HYBRID,
            zoom: defaultZoom,
            zoomControl: true,
            zoomControlOptions: {
                style: naver.maps.ZoomControlStyle.SMALL,
                position: naver.maps.Position.TOP_RIGHT,
            },
            mapTypeControl: true,
            mapTypeControlOptions: {
                style: naver.maps.MapTypeControlStyle.BUTTON
            }
        }
        map = new naver.maps.Map(mapRef.current, mapOpts);
        map!.mapTypes.set("osm", openStreetMapType);
    }

    const initializeRobotMarker: Function = (): void => {
        const initialCurrMarker: naver.maps.Marker = new naver.maps.Marker({
            position: map!.getCenter(),
            map: map,
            title: "RobotCurrentPos",
            icon: {
                url: process.env.PUBLIC_URL + "marker_current_position.png",
                size: new naver.maps.Size(35, 35),
                scaledSize: new naver.maps.Size(35, 35),
                origin: new naver.maps.Point(0, 0),
                anchor: new naver.maps.Point(12, 34)
            },
            zIndex: 1000,
            clickable: false
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

        const marker: naver.maps.Marker = new naver.maps.Marker({
            position: new naver.maps.LatLng(node.position.latitude, node.position.longitude),
            map: map,
            title: `${node.node_id}/${node.kind}/${node.heading}/${node.driving_option}/${node.direction}`,
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
        return function (e: any) {
            const marker: naver.maps.Marker = pathMarkerArray[seq];
            const infoWindow: naver.maps.InfoWindow = pathInfoWindowarray[seq];

            if (infoWindow.getMap()) {
                infoWindow.close();
            } else {
                infoWindow.open(map!, marker);
            }
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
            initializeMap();
        } else return;

        initializeRobotMarker();

        if (mapRef.current && naver && map) {
            drawPathMarker();
            drawPathPolyline();
            setDefaultZoom(21);

            if (currMarker) {
                changeMapCenter(currMarker!.getPosition());
            }
        } else return;
    }, [naver, map, pathData]);

    useEffect((): void => {
        if (gpsData) {
            setCurrentGps({
                latitude: parseFloat(gpsData.latitude.toFixed(7)),
                longitude: parseFloat(gpsData.longitude.toFixed(7))
            });

            currMarker!.setPosition(new naver.maps.LatLng(gpsData.latitude, gpsData.longitude));
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

    useEffect(() => {
        return () => {
            if (map) {
                setCurrMarker(null);
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
                        <h3>GPS</h3>
                        <div className="">
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
                            <div className="route_status_current_route">
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
                        <div className="">
                            <button className={"route_btn_request route_btn_can_init"} onClick={onInitClick}>CAN 초기화</button>
                            <button className={"route_btn_request route_btn_emergency_stop"} onClick={onEmergencyStopClick}>비상 정지</button>
                            <button className={"route_btn_request route_btn_emergency_stop"} onClick={onGoalCancelClick}>주행 취소</button>
                            <button className={"route_btn_request route_btn_emergency_resume"} onClick={onEmergencyResumeClick}>재개</button>
                        </div>
                    </div>
                </div>
            </div>
        </div>
    );
};

export default MapComponent;