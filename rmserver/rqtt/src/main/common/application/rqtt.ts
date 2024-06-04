import fs from "fs";
import mqtt from "mqtt";
import os from "os";
import path from "path";
import * as rclnodejs from "rclnodejs";
import util from "util";
import * as configFilePathsJSON from "../config/configFilePaths.json";

const LOG_MQTT_TAG: string = "[MQTT]";

export default class Rqtt {
    private logger: rclnodejs.Logging;
    private mqttConnectionInfo: any;
    private client: mqtt.MqttClient | null | undefined;

    constructor(node: rclnodejs.Node) {
        this.logger = node.getLogger();
    }

    private async loadConfig(): Promise<any> {
        const homeDir: string = os.homedir();
        const mqttConfigFilePath: string = path.join(homeDir, configFilePathsJSON.mqtt_config_file_path);
        const mqttConfigData: any = JSON.parse(await util.promisify(fs.readFile)(mqttConfigFilePath, "utf8"));
        return mqttConfigData;
    }

    public async initialize(): Promise<mqtt.MqttClient> {
        this.mqttConnectionInfo = await this.loadConfig();
        this.logger.info(`${LOG_MQTT_TAG} connectionInfo: ${JSON.stringify(this.mqttConnectionInfo)}`);

        const type: string = this.mqttConnectionInfo.type === "websockets" ? "ws" : "mqtt";
        const brokerURL: string = `${type}://${this.mqttConnectionInfo.host}:${this.mqttConnectionInfo.port}${this.mqttConnectionInfo.path}`;
        this.client = mqtt.connect(brokerURL);

        return new Promise<mqtt.MqttClient>((resolve, reject) => {
            this.client?.on("connect", () => {
                this.logger.info(`${LOG_MQTT_TAG} Connected to MQTT broker at ${brokerURL}`);
                resolve(this.client!);
            });
    
            this.client?.on("error", (error) => {
                this.logger.error(`${LOG_MQTT_TAG} MQTT connection error: ${error.message}`);
                reject(null);
            });
        });
    }

    public publish(rqttC: mqtt.MqttClient, topic: string, message: string): void {
        try {
            rqttC.publish(topic, message);
        } catch (error: any) {
            this.logger.error(`${LOG_MQTT_TAG} publishing error: ${error}`);
            throw new Error(error);
        }
    }

    public subscribe(rqttC: mqtt.MqttClient, topic: string): void {
        try {
            rqttC.subscribe(topic, (err, granted) => {
                if (err) {
                    this.logger.error(`${LOG_MQTT_TAG} ${topic} subscription error: ${err.message}`);
                } else {
                    this.logger.info(`${LOG_MQTT_TAG} subscription granted: ${granted?.map(g => g.topic).join(", ")}`);
                }
            });
        } catch (error: any) {
            this.logger.error(`${LOG_MQTT_TAG} Error in subscribe method: ${error}`);
        }
    }

    public addSubscriptionCallback(rqttC: mqtt.MqttClient, topic: string, callback: (message: any) => void): void {
        try {
            rqttC.on("message", (receivedTopic: string, message: Buffer) => {
                if (topic === receivedTopic) {
                    callback(JSON.parse(message.toString()));
                } else return;
            });
        } catch (error: any) {
            this.logger.error(`${LOG_MQTT_TAG} Error in addSubscriptionCallback method: ${error}`);
        }
    }
}
