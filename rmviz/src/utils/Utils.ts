import MqttClient from "../api/mqttClient";
import { useMediaQuery } from "react-responsive";

export const KTP_DEV_ID: string = "KECDSEMITB001";

export function getCurrentTime(): string {
    const now: Date = new Date();
    const year: string = now.getFullYear().toString().slice(-2);
    const month: string = ('0' + (now.getMonth() + 1)).slice(-2);
    const date: string = ('0' + now.getDate()).slice(-2);
    const hours: string = ('0' + now.getHours()).slice(-2);
    const minutes: string = ('0' + now.getMinutes()).slice(-2);
    const seconds: string = ('0' + now.getSeconds()).slice(-2);
    const milliseconds: string = ('00' + now.getMilliseconds()).slice(-3);

    const currentTime: string = year + month + date + hours + minutes + seconds + milliseconds;
    return currentTime;
}

export const filterJSON = (json: any): any => {
    const { default: removedDefault, ...newData } = json;

    return newData;
}

export const onClickMqttPublish = (mqttClient: MqttClient, topic: string, json: any): void => {
    const filteredJSON: any = filterJSON(json);
    const stringifiedJSON: string = JSON.stringify(filteredJSON);
    console.info(`${topic} publish with : ${stringifiedJSON}`);
    mqttClient!.publish(topic, stringifiedJSON);
}

export function useDevice() {
    const isDesktopOrLaptop = useMediaQuery({ minDeviceWidth: 1224 });
    const isBigScreen = useMediaQuery({ minDeviceWidth: 1824 });
    const isTabletOrMobile = useMediaQuery({ maxWidth: 1224 });
    const isTabletOrMobileDevice = useMediaQuery({ maxDeviceWidth: 1224 });
    const isPortrait = useMediaQuery({ orientation: 'portrait' });
    const isRetina = useMediaQuery({ minResolution: '2dppx' });

    return {
        isDesktopOrLaptop,
        isBigScreen,
        isTabletOrMobile,
        isTabletOrMobileDevice,
        isPortrait,
        isRetina
    };
}