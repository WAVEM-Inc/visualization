import $ from "jquery";
import { useEffect, useRef, useState } from "react";
import "./MapComponents.css";

interface MapComponentProps {
    pathData: any;
    gpsData: any;
    center: naver.maps.LatLng;
    odomEularData: any;
}

const MapComponent = ({ center, pathData, gpsData, odomEularData }: MapComponentProps) => {
    const { naver } = window;
    const mapRef: React.MutableRefObject<HTMLDivElement | null> = useRef<HTMLDivElement>(null);

    let map: naver.maps.Map | null = null;
    const [currMarker, setCurrMarker] = useState<naver.maps.Marker | null>(null);
    const [currentGps, setCurrentGps] = useState<any>({
        latitude: 0.0,
        longitude: 0.0
    });
    const [currentOdomEular, setCurrentOdomEular] = useState<number>();

    let pathMarkerArray: Array<naver.maps.Marker> = [];
    let pathInfoWindowarray: Array<naver.maps.InfoWindow> = [];

    const [currentMode, setCurrentMode] = useState<string>("KEC");
    const [defaultZoom, setDefaultZoom] = useState<number>(19);

    const wkValveCoord: naver.maps.LatLng = new naver.maps.LatLng(35.157851, 128.858111);
    const blueSpaceCoord: naver.maps.LatLng = new naver.maps.LatLng(37.305985, 127.2401652);
    const kecCoord: naver.maps.LatLng = new naver.maps.LatLng(36.1137155, 128.3676005);

    const initializeMap = (): void => {
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

        naver.maps.Event.once(map, 'init', function () {
            const controlHtml: string =
                `
            <div style="position: absolute; z-index: 100; margin: 0px; padding: 0px; pointer-events: none; top: 0px;">
                <div style="border: 0px; margin: 0px; padding: 0px; pointer-events: none; float: left;"">
                    <div style="position: relative; z-index: 69; margin: 10px; pointer-events: auto;">
                        <ul class="move_btn_ul">
                            <li class="move_btn_li"><button class="move_btn move_btn_wk">부산 원광밸브</button></li>
                            <li class="move_btn_li"><button class="move_btn move_btn_bs">용인 블루스페이스</button></li>
                            <li class="move_btn_li"><button class="move_btn move_btn_kec">KEC 구미</button></li>
                        </ul>
                    </div>
                </div>
            </div>
            `;
            const moveControls: naver.maps.CustomControl = new naver.maps.CustomControl(controlHtml, {
                position: naver.maps.Position.TOP_LEFT
            });
            moveControls.setMap(map);

            naver.maps.Event.addDOMListener(moveControls.getElement(), 'click', function (e: Event) {
                const target: HTMLElement = e.target as HTMLElement;
                const locationName: string = target.textContent || "";

                switch (locationName.trim()) {
                    case "부산 원광밸브":
                        setCurrentMode("WkValve");
                        setDefaultZoom(19);
                        map!.setZoom(19);
                        map!.setCenter(wkValveCoord);
                        break;
                    case "용인 블루스페이스":
                        setCurrentMode("BlueSpace");
                        setDefaultZoom(map!.getMaxZoom());
                        map!.setZoom(map!.getMaxZoom());
                        map!.setCenter(blueSpaceCoord);
                        break;
                    case "KEC 구미":
                        setCurrentMode("KEC");
                        setDefaultZoom(19);
                        map!.setZoom(19);
                        map!.setCenter(kecCoord);
                        break;
                    default:
                        break;
                }
            });
        });
    }

    const initializeRobotMarker = (): void => {
        const initialCurrMarker: naver.maps.Marker = new naver.maps.Marker({
            position: map!.getCenter(),
            map: map,
            title: "RobotCurrentPos",
            icon: {
                url: process.env.PUBLIC_URL + "free-icon-gps-12795350.png",
                size: new naver.maps.Size(35, 35),
                scaledSize: new naver.maps.Size(35, 35),
                origin: new naver.maps.Point(0, 0),
                anchor: new naver.maps.Point(12, 34)
            },
            zIndex: 1000
        });
        setCurrMarker(initialCurrMarker);
    }

    const addPathMarker = (node: any): any => {
        console.info(`addPathMarker node : ${JSON.stringify(node)}`);

        const marker: naver.maps.Marker = new naver.maps.Marker({
            position: new naver.maps.LatLng(node.position.latitude, node.position.longitude),
            map: map,
            title: `${node.node_id}/${node.kind}`,
            icon: {
                url: process.env.PUBLIC_URL + "free-icon-location-7009904.png",
                size: new naver.maps.Size(30, 30),
                scaledSize: new naver.maps.Size(30, 30),
                origin: new naver.maps.Point(0, 0),
                anchor: new naver.maps.Point(12, 34)
            }
        });
        

        return marker;
    }

    const getClickHandler = (seq: number): Function => {
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

    const drawPathPolyline = (): void => {
        if (pathData) {
            const nodeList: Array<any> = pathData.node_list;

            const uniqueNodeIds: Set<string> = new Set();
            for (const node of nodeList) {
                uniqueNodeIds.add(node.start_node.node_id);
                uniqueNodeIds.add(node.end_node.node_id);
            }

            for (const nodeId of uniqueNodeIds) {
                const node = nodeList.find((node) => node.start_node.node_id === nodeId || node.end_node.node_id === nodeId);
                if (node) {
                    pathMarkerArray.push(addPathMarker(node.start_node));
                    pathMarkerArray.push(addPathMarker(node.end_node));
                }
            }

            for (var i = 0, ii = pathMarkerArray.length; i < ii; i++) {
                naver.maps.Event.addListener(pathMarkerArray[i], "click", getClickHandler(i));
            }

            let path: Array<any> = [];
            for (const pathMarker of pathMarkerArray) {
                const contentString: string = [
                    '<div class="iw_inner">',
                    `   <h3>ID : ${pathMarker!.getTitle().split("/")[0]}</h3>`,
                    `   <p>종류 : ${pathMarker!.getTitle().split("/")[1]}</p>`,
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
                strokeOpacity: 1.0,
                strokeWeight: 6.0
            });
        }
    }

    useEffect(() => {
        if (mapRef.current && naver && !map) {
            initializeMap();
        } else return;

        initializeRobotMarker();

        if (mapRef.current && naver && map) {
            drawPathPolyline();
        }
    }, [naver, map, pathData]);

    useEffect(() => {
        if (gpsData) {
            setCurrentGps({
                latitude: parseFloat(gpsData.latitude.toFixed(7)),
                longitude: parseFloat(gpsData.longitude.toFixed(7))
            });

            currMarker!.setPosition(new naver.maps.LatLng(gpsData.latitude, gpsData.longitude));
        }
    }, [gpsData]);

    useEffect(() => {
        if (odomEularData) {
            setCurrentOdomEular(parseFloat(odomEularData.pose.orientation.y.toFixed(2)));
        }

        if (currentOdomEular) {
            const angle: number = 360 - currentOdomEular!;
            $("div[title|='RobotCurrentPos'").css("transform", `rotate(${angle}deg)`);
        }
        
    }, [odomEularData]);

    useEffect(() => {
        if (mapRef.current) {
            console.info(`currentMode : ${currentMode}`);
            const moveControlsElement = mapRef.current.querySelector('.move_btn_ul');

            if (moveControlsElement) {
                console.info(`li`);
                const li = moveControlsElement.querySelector('.move_btn_li');

                if (li) {
                    switch (li.textContent!.trim()) {
                        case "부산 원광밸브":
                            moveControlsElement.classList.add('active');
                            break;
                        default:
                            break;
                    }
                    if (li.textContent!.trim() === currentMode) {
                        moveControlsElement.classList.add('active');
                    } else {
                        moveControlsElement.classList.remove('active');
                    }
                }
            }
        }
    }, [currentMode]);

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
                        longitude : {currentGps.longitude}
                        <br></br>
                        latitude : {currentGps.latitude}
                    </div>
                    <div className={"odom_eular_data_container"}>
                        <h3>Odom Eular</h3>
                        angle : {currentOdomEular}
                    </div>
                </div>
            </div>
        </div>
    );
};

export default MapComponent;