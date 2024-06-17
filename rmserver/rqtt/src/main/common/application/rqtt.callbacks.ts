
export let rtmDataProcessCallback: (topic: string, data: any) => void = () => {

}

export function setRtmDataProcessCallback(callback: (topic: string, data: any) => void) {
    rtmDataProcessCallback = callback;
}