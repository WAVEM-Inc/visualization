import proj4 from "proj4";

const EARTH_RADIUS = 6378137;

function toDegrees(radians: number): number {
    return radians * (180 / Math.PI);
}

export function calculateOffset(lat: number, lon: number, distance: number, bearing: number): { lat: number, lon: number } {
    const angularDistance = distance / EARTH_RADIUS;
    const bearingRad = toRadians(bearing);

    const lat1 = toRadians(lat);
    const lon1 = toRadians(lon);

    const lat2 = Math.asin(
        Math.sin(lat1) * Math.cos(angularDistance) +
        Math.cos(lat1) * Math.sin(angularDistance) * Math.cos(bearingRad)
    );

    const lon2 = lon1 + Math.atan2(
        Math.sin(bearingRad) * Math.sin(angularDistance) * Math.cos(lat1),
        Math.cos(angularDistance) - Math.sin(lat1) * Math.sin(lat2)
    );

    return {
        lat: toDegrees(lat2),
        lon: toDegrees(lon2)
    };
}

export function calculateBearing(lat1: number, lon1: number, lat2: number, lon2: number): number {
    const lat1Rad = toRadians(lat1);
    const lon1Rad = toRadians(lon1);
    const lat2Rad = toRadians(lat2);
    const lon2Rad = toRadians(lon2);

    const dLon = lon2Rad - lon1Rad;

    const y = Math.sin(dLon) * Math.cos(lat2Rad);
    const x = Math.cos(lat1Rad) * Math.sin(lat2Rad) -
              Math.sin(lat1Rad) * Math.cos(lat2Rad) * Math.cos(dLon);

    let bearingRad = Math.atan2(y, x);
    let bearingDeg = toDegrees(bearingRad);
    
    // 방위각이 0~360도 사이로 나오도록 보정
    bearingDeg = (bearingDeg + 360) % 360;

    return bearingDeg;
}

const toRadians: Function = (degrees: number): number => {
    return degrees * (Math.PI / 180);
}

const getUTMZone: Function = (longitude: number): number => {
    return Math.floor((longitude + 180) / 6) + 1;
}

const utmProj = (zone: number, north: boolean) => `+proj=utm +zone=${zone} +${north ? "north" : "south"} +ellps=WGS84 +datum=WGS84 +units=m +no_defs`;

export const calculateVertices: Function = (node: any): Array<google.maps.LatLng> => {
    const latlngArray: Array<google.maps.LatLng> = [];
    const range: any = node.detectionRange[0];

    const zone: number = getUTMZone(node.position.longitude);
    const north: boolean = node.position.latitude >= 0;

    const utmProjString: string = utmProj(zone, north);

    const [nodeX, nodeY]: [any, any] = proj4("WGS84", utmProjString, [node.position.longitude, node.position.latitude]);

    const x1: number = nodeX - range.widthLeft
    const y1: number = nodeY + range.offset;

    const x2: number = nodeX + range.widthRight;
    const y2: number = y1;

    const x3: number = x2;
    const y3: number = y1 + range.height;

    const x4: number = x1;
    const y4: number = y3;

    const radian: number = toRadians(node.heading);

    const rotate: Function = (x: number, y: number): any => ({
        x: Math.cos(radian) * (x - nodeX) - Math.sin(radian) * (y - nodeY) + nodeX,
        y: Math.sin(radian) * (x - nodeX) + Math.cos(radian) * (y - nodeY) + nodeY
    });

    const { x: x1p, y: y1p }: any = rotate(x1, y1);
    console.info(`{ x: x1p, y: y1p } : ${JSON.stringify({ x: x1p, y: y1p })}`);

    const { x: x2p, y: y2p }: any = rotate(x2, y2);
    console.info(`{ x: x2p, y: y2p } : ${JSON.stringify({ x: x2p, y: y2p })}`);

    const { x: x3p, y: y3p }: any = rotate(x3, y3);
    console.info(`{ x: x3p, y: y3p } : ${JSON.stringify({ x: x3p, y: y3p })}`);

    const { x: x4p, y: y4p }: any = rotate(x4, y4);
    console.info(`{ x: x4p, y: y4p } : ${JSON.stringify({ x: x4p, y: y4p })}`);

    try {
        const [lng1, lat1]: [any, any] = proj4(utmProjString, "WGS84", [x1p, y1p]);
        const [lng2, lat2]: [any, any] = proj4(utmProjString, "WGS84", [x2p, y2p]);
        const [lng3, lat3]: [any, any] = proj4(utmProjString, "WGS84", [x3p, y3p]);
        const [lng4, lat4]: [any, any] = proj4(utmProjString, "WGS84", [x4p, y4p]);

        if (
            Number.isFinite(lat1) && Number.isFinite(lng1) &&
            Number.isFinite(lat2) && Number.isFinite(lng2) &&
            Number.isFinite(lat3) && Number.isFinite(lng3) &&
            Number.isFinite(lat4) && Number.isFinite(lng4)
        ) {
            latlngArray.push(new google.maps.LatLng({ lat: lat1, lng: lng1 }));
            latlngArray.push(new google.maps.LatLng({ lat: lat2, lng: lng2 }));
            latlngArray.push(new google.maps.LatLng({ lat: lat3, lng: lng3 }));
            latlngArray.push(new google.maps.LatLng({ lat: lat4, lng: lng4 }));
        } else {
            throw new Error("Invalid coordinates");
        }
    } catch (error) {
        console.error("Error converting coordinates:", error);
    }

    return latlngArray;
}