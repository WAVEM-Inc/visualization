
export const initializeMap: Function = (mapDiv: string | HTMLElement, center: naver.maps.LatLng): naver.maps.Map | null => {
    const defaultZoom: number = 21;

    const mapOpts: any = {
        center: center,
        mapTypeId: naver.maps.MapTypeId.HYBRID,
        zoom: defaultZoom,
        zoomControl: true,
        zoomControlOptions: {
            style: naver.maps.ZoomControlStyle.SMALL,
            position: naver.maps.Position.TOP_RIGHT
        },
        mapTypeControl: true,
        mapTypeControlOptions: {
            style: naver.maps.MapTypeControlStyle.BUTTON,
            position: naver.maps.Position.TOP_RIGHT
        }
    }

    const map: naver.maps.Map | null = new naver.maps.Map(mapDiv, mapOpts);

    return map;
}

export const initializeRobotMarker: Function = (map: naver.maps.Map): naver.maps.Marker => {
    console.info(`PUBLIC_URL : ${process.env.PUBLIC_URL}`);
    const robotMarker: naver.maps.Marker = new naver.maps.Marker({
        position: map.getCenter(),
        map: map,
        title: "RobotCurrentPos",
        icon: {
            url: process.env.PUBLIC_URL + "../marker_current_position.png",
            size: new naver.maps.Size(35, 35),
            scaledSize: new naver.maps.Size(35, 35),
            origin: new naver.maps.Point(0, 0),
            anchor: new naver.maps.Point(12, 34)
        },
        zIndex: 1000,
        clickable: false
    });

    return robotMarker;
}

export const initializeRobotFilteredMarker: Function = (map: naver.maps.Map): naver.maps.Marker => {
    const robotMarker: naver.maps.Marker = new naver.maps.Marker({
        position: map.getCenter(),
        map: map,
        title: "RobotCurrentPos",
        icon: {
            url: process.env.PUBLIC_URL + "../marker_filtered.png",
            size: new naver.maps.Size(30, 30),
            scaledSize: new naver.maps.Size(30, 30),
            origin: new naver.maps.Point(0, 0),
            anchor: new naver.maps.Point(10, 32)
        },
        zIndex: 1000,
        clickable: false
    });

    return robotMarker;
}

export const addGPSInitMarkerControl: Function = (map: naver.maps.Map, initializeGPSInitMarker: naver.maps.Marker): void => {
    const gpsInitBtnHTML = [
        '<button>',
        '<span class="spr_trff spr_ico_mylct">GPS 초기화 마커 추가</span>',
        "</button>"
    ].join("");

    naver.maps.Event.once(map, 'init', function () {
        const customControl = new naver.maps.CustomControl(gpsInitBtnHTML, {
            position: naver.maps.Position.TOP_LEFT
        });

        customControl.setMap(map);

        naver.maps.Event.addDOMListener(customControl.getElement(), 'click', function () {
            initializeGPSInitMarker.setMap(map);
        });
    });
}

export const initializeGPSInitMarker: Function = (map: naver.maps.Map): naver.maps.Marker => {
    const gpsInitMarker: naver.maps.Marker = new naver.maps.Marker({
        map: map,
        position: map.getCenter(),
        title: "GPSInitMarker",
        zIndex: 1000,
        icon: {
            url: process.env.PUBLIC_URL + "../marker_filtered.png",
            size: new naver.maps.Size(30, 30),
            scaledSize: new naver.maps.Size(30, 30),
            origin: new naver.maps.Point(0, 0),
            anchor: new naver.maps.Point(10, 32)
        }
    });

    return gpsInitMarker;
}