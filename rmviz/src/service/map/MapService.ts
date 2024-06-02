import { faLocationArrow } from "@fortawesome/free-solid-svg-icons";
import { calculateVertices } from "../math/MathService";

export const initializeMap: Function = (mapElement: HTMLElement, center: google.maps.LatLng): google.maps.Map => {
    const mapInstance: google.maps.Map = new google.maps.Map(mapElement, {
        center: center,
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

    return mapInstance;
}

export const initializeRobotMarker: Function = (map: google.maps.Map): google.maps.Marker => {
    const robotMarker: google.maps.Marker = new google.maps.Marker({
        position: map.getCenter(),
        map: map!,
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

    return robotMarker;
}

export const initializeKECDBorderLine: Function = (map: google.maps.Map): void => {
    const polyline: google.maps.Polyline = new google.maps.Polyline({
        map: map,
        path: [
            new google.maps.LatLng(36.11387, 128.3687),
            new google.maps.LatLng(36.11372, 128.3687)
        ],
        clickable: true,
        strokeColor: "red",
        strokeOpacity: 1.0,
        strokeWeight: 5.0
    });
}

export const changeMapCenter: Function = (map: google.maps.Map, coord: google.maps.LatLng): void => {
    map.setCenter(coord);
}

export const updateRobotMakerIcon: Function = (currRobotMarker: google.maps.Marker, angle: number): void => {
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

export const addPathMarker: Function = (
    map: google.maps.Map,
    node: any,
    is_start: boolean,
    is_end: boolean
): google.maps.Marker => {
    let iconUrl: string = "";

    if (is_start && !is_end) {
        iconUrl = process.env.PUBLIC_URL + "../marker_start.png";
    } else if (!is_start && is_end) {
        iconUrl = process.env.PUBLIC_URL + "../marker_arrive.png";
    } else if (node.kind === "intersection") {
        iconUrl = process.env.PUBLIC_URL + "../marker_intersection.png";
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
        map: map,
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

const getInfoWindowClickHandler: Function = (
    map: google.maps.Map,
    pathMarkerArray: Array<google.maps.Marker>,
    pathInfoWindowarray: Array<google.maps.InfoWindow>,
    seq: number
): Function => {
    let isOpened: number = 0;

    return function (e: any) {
        const marker: google.maps.Marker = pathMarkerArray[seq];
        const infoWindow: google.maps.InfoWindow = pathInfoWindowarray[seq];

        if (isOpened === 0) {
            infoWindow.open(map!, marker);
            isOpened++;
        } else {
            infoWindow.close();
            isOpened = 0;
        }
    }
}

export const addPathPolyline: Function = (
    map: google.maps.Map,
    pathMarkerArray: Array<google.maps.Marker>,
    pathInfoWindowarray: Array<google.maps.InfoWindow>
): google.maps.Polyline => {

    for (let i = 0, ii = pathMarkerArray.length; i < ii; i++) {
        google.maps.event.addListener(
            pathMarkerArray[i],
            "click",
            getInfoWindowClickHandler(
                map,
                pathMarkerArray,
                pathInfoWindowarray,
                i
            )
        );
    }

    let path: Array<any> = [];
    for (const pathMarker of pathMarkerArray) {
        const contentString: string = [
            "<div class='node_info_window'>",
            `   <h3>ID : ${pathMarker!.getTitle()!.split("/")[0]}</h3>`,
            `   <p>종류 : ${pathMarker!.getTitle()!.split("/")[1]}</p>`,
            `   <p>진출 각도 : ${pathMarker!.getTitle()!.split("/")[2]}</p>`,
            `   <p>주행 옵션 : ${pathMarker!.getTitle()!.split("/")[3]}</p>`,
            `   <p>주행 방향 : ${pathMarker!.getTitle()!.split("/")[4]}</p>`,
            `   <p>경도 : ${pathMarker!.getPosition()!.lng()}</p>`,
            `   <p>위도 : ${pathMarker!.getPosition()!.lat()}</p>`,
            "</div>"
        ].join("");

        const infoWindow: google.maps.InfoWindow = new google.maps.InfoWindow({
            content: contentString
        });

        pathInfoWindowarray.push(infoWindow);

        for (let i = 0, ii = pathMarkerArray.length; i < ii; i++) {
            google.maps.event.addListener(
                pathMarkerArray[i],
                "click",
                getInfoWindowClickHandler(
                    map,
                    pathMarkerArray,
                    pathInfoWindowarray,
                    i
                )
            );
        }
        path.push(pathMarker.getPosition());
    }

    const polyline: google.maps.Polyline = new google.maps.Polyline({
        map: map,
        path: path,
        clickable: true,
        strokeColor: "yellow",
        strokeOpacity: 1.0,
        strokeWeight: 5.0
    });

    return polyline;
}

export const addDetectionRangePolygon: Function = (
    map: google.maps.Map,
    nodeList: Array<any>
): Array<google.maps.Polygon> => {
    const detectionRangePolygonArray: Array<google.maps.Polygon> = [];
    const nonEmptyDetectionRanges: any[] = nodeList.filter(node => Array.from(node.detectionRange).length > 0);

    for (const d of nonEmptyDetectionRanges) {
        console.info(`d : ${JSON.stringify(d)}`);
        const vertices: any[] = calculateVertices(d);
        vertices.filter(vertice => console.log(`vertice : ${JSON.stringify(vertice)}`));

        const polygon: google.maps.Polygon = new google.maps.Polygon({
            map: map,
            paths: vertices,
            strokeColor: "#FF0000",
            strokeOpacity: 0.8,
            strokeWeight: 2,
            fillColor: "#FF0000",
            fillOpacity: 0.35
        });
        detectionRangePolygonArray.push(polygon);
    }

    return detectionRangePolygonArray;
}