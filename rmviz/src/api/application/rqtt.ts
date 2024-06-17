import mqtt from "mqtt";

const LOG_MQTT_TAG: string = "[MQTT]";

export default class Rqtt {
    
    private _mqttConnectionInfo: any;
    private _client: mqtt.MqttClient | null | undefined;

    constructor() {
    }

    public initialize(mqttConnectionInfo: any): mqtt.MqttClient {
        this._mqttConnectionInfo = mqttConnectionInfo
        console.info(`${LOG_MQTT_TAG} connectionInfo: ${JSON.stringify(this._mqttConnectionInfo)}`);

        const type: string = this._mqttConnectionInfo.type === "websockets" ? "ws" : "mqtt";
        const brokerURL: string = `${type}://${this._mqttConnectionInfo.host}:${this._mqttConnectionInfo.port}${this._mqttConnectionInfo.path}`;
        this._client = mqtt.connect(brokerURL);

        this._client?.on("connect", (): void => {
            console.info(`${LOG_MQTT_TAG} Connected to MQTT broker at ${brokerURL}`);
        });

        this._client?.on("error", (error): void => {
            console.error(`${LOG_MQTT_TAG} MQTT connection error: ${error.message}`);
        });

        this._client?.on("disconnect", (): void => {
            console.warn(`${LOG_MQTT_TAG} MQTT disconnected...`);
        });

        return this._client;
    }

    public publish(rqttC: mqtt.MqttClient, topic: string, message: string): void {
        try {
            rqttC.publish(topic, message);
        } catch (error: any) {
            console.error(`${LOG_MQTT_TAG} publishing error: ${error}`);
            throw new Error(error);
        }
    }

    public subscribe(rqttC: mqtt.MqttClient, topic: string): void {
        try {
            rqttC.subscribe(topic, (err, granted) => {
                if (err) {
                    console.error(`${LOG_MQTT_TAG} ${topic} subscription error: ${err.message}`);
                } else {
                    console.info(`${LOG_MQTT_TAG} subscription granted: ${granted?.map(g => g.topic).join(", ")}`);
                }
            });
        } catch (error: any) {
            console.error(`${LOG_MQTT_TAG} Error in subscribe method: ${error}`);
        }
    }

    public addSubscriptionCallback(rqttC: mqtt.MqttClient, topic: string, callback: (message: any) => void): void {
        try {
            rqttC.on("message", (receivedTopic: string, message: Buffer): void => {
                if (topic === receivedTopic) {
                    callback(JSON.parse(message.toString()));
                } else return;
            });
        } catch (error: any) {
            console.error(`${LOG_MQTT_TAG} Error in addSubscriptionCallback method: ${error}`);
        }
    }

    public disconnect(): void {
        try {
            this._client?.end();
        } catch (error: any) {
            console.error(`${LOG_MQTT_TAG} Error in disconnect method: ${error}`);
        }
    }
}
