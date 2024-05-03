export interface MapState {
    path: any | null;
    gps: any;
    gpsFiltered: any;
    routeStatus: any | null;
    odomEular: any;
}

export const initialMapState: MapState = {
    path: {},
    gps: {
        header: {},
        status: {
            status: 0,
            service: 0
        },
        longitude: 0.0,
        latitude: 0.0
    },
    gpsFiltered: {
        header: {},
        status: {
            status: 0,
            service: 0
        },
        longitude: 0.0,
        latitude: 0.0
    },
    routeStatus: {},
    odomEular: {}
};

type MapStateAction = {
    type: string,
    payload: any
} | {
    type: string,
    payload: any
} | {
    type: string,
    payload: any
} | {
    type: string,
    payload: any
} | {
    type: string,
    payload: any
}

export const SET_PATH: string = "SET_PATH";
export const SET_GPS: string = "SET_GPS";
export const SET_GPS_FILTERED: string = "SET_GPS_FILTERED";
export const SET_ROUTE_STATUS: string = "SET_ROUTE_STATUS";
export const SET_ODOM_EULAR: string = "SET_ODOM_EULAR";

export function mapStateReducer(state: MapState, action: MapStateAction): MapState {
    switch (action.type) {
        case SET_PATH:
            return {
                ...state,
                path: action.payload
            };
        case SET_GPS:
            return {
                ...state,
                gps: action.payload
            };
        case SET_GPS_FILTERED:
            return {
                ...state,
                gpsFiltered: action.payload
            };
        case SET_ROUTE_STATUS:
            return {
                ...state,
                routeStatus: action.payload
            };
        case SET_ODOM_EULAR:
            return {
                ...state,
                odomEular: action.payload
            };
        default:
            return state;
    }
}