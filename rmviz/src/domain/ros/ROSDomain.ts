export interface ROSState {
    urdf: any;
}

export const initalROSState: ROSState = {
    urdf: {}
}

type ROSStateAction = {
    type: string,
    payload: any
}

export const SET_URDF = "SET_URDF";

export function rosStateReducer(state: ROSState, action: ROSStateAction): ROSState {
    switch (action.type) {
        case SET_URDF:
            return {
                ...state,
                urdf: action.payload
            };
        default:
            return state;
    }
}