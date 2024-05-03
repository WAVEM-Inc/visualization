export interface TopState {
    heartBeat: any | null;
    battery: any;
}

export const initialTopState: TopState = {
    heartBeat: {
        ping_timer: 0.0
    },
    battery: {
        voltage: 0.0
    }
};

export const SET_HEARTBEAT: string = "SET_HEARTBEAT";
export const SET_BATTERY: string = "SET_BATTERY";

type TopStateAction = { type: string, payload: any } | { type: string, payload: any };

export function topStateReducer(state: TopState, action: TopStateAction): TopState {
    switch (action.type) {
        case SET_HEARTBEAT:
            return { ...state, heartBeat: action.payload };
        case SET_BATTERY:
            return { ...state, battery: action.payload };
        default:
            return state;
    }
}