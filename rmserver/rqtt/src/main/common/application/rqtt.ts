import fs from "fs";
import mqtt from "mqtt";
import os from "os";
import path from "path";
import * as rclnodejs from "rclnodejs";
import util from "util";
import * as configFilePathsJSON from "../config/configFilePaths.json";

const LOG_MQTT_TAG: string = "[MQTT]";

export default class Rqtt {
    private _logger: rclnodejs.Logging;
    private _mqttConnectionInfo: any;
    private _client: mqtt.MqttClient | null | undefined;

    constructor(node: rclnodejs.Node) {
        this._logger = node.getLogger();
    }

    private async loadConfig(): Promise<any> {
        const homeDir: string = os.homedir();
        const mqttConfigFilePath: string = path.join(homeDir, configFilePathsJSON.mqtt_config_file_path);
        const mqttConfigData: any = JSON.parse(await util.promisify(fs.readFile)(mqttConfigFilePath, "utf8"));
        return mqttConfigData;
    }

    public async initialize(): Promise<mqtt.MqttClient> {
        this._mqttConnectionInfo = await this.loadConfig();
        this._logger.info(`${LOG_MQTT_TAG} connectionInfo: ${JSON.stringify(this._mqttConnectionInfo)}`);

        const type: string = this._mqttConnectionInfo.type === "websockets" ? "ws" : "mqtt";
        const brokerURL: string = `${type}://${this._mqttConnectionInfo.host}:${this._mqttConnectionInfo.port}${this._mqttConnectionInfo.path}`;
        this._client = mqtt.connect(brokerURL);

        return new Promise<mqtt.MqttClient>((resolve, reject): void => {
            this._client?.on("connect", (): void => {
                this._logger.info(`${LOG_MQTT_TAG} Connected to MQTT broker at ${brokerURL}`);
                resolve(this._client!);
            });
    
            this._client?.on("error", (error): void => {
                this._logger.error(`${LOG_MQTT_TAG} MQTT connection error: ${error.message}`);
                reject(null);
            });

            this._client?.on("disconnect", (): void => {
                this._logger.warn(`${LOG_MQTT_TAG} MQTT disconnected...`);
                reject(null);
            });
        });
    }

    public publish(rqttC: mqtt.MqttClient, topic: string, message: string): void {
        try {
            rqttC.publish(topic, message);
        } catch (error: any) {
            this._logger.error(`${LOG_MQTT_TAG} publishing error: ${error}`);
            throw new Error(error);
        }
    }

    public subscribe(rqttC: mqtt.MqttClient, topic: string): void {
        try {
            rqttC.subscribe(topic, (err, granted) => {
                if (err) {
                    this._logger.error(`${LOG_MQTT_TAG} ${topic} subscription error: ${err.message}`);
                } else {
                    this._logger.info(`${LOG_MQTT_TAG} subscription granted: ${granted?.map(g => g.topic).join(", ")}`);
                }
            });
        } catch (error: any) {
            this._logger.error(`${LOG_MQTT_TAG} Error in subscribe method: ${error}`);
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
            this._logger.error(`${LOG_MQTT_TAG} Error in addSubscriptionCallback method: ${error}`);
        }
    }

    public disconnect(): void {
        try {
            this._client?.end();
        } catch (error: any) {
            this._logger.error(`${LOG_MQTT_TAG} Error in disconnect method: ${error}`);
        }
    }
}
