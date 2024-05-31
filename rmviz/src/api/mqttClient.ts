import mqtt, { IClientOptions } from 'mqtt';

export default class MqttClient {

    broker_url: string;
    connect_opts: IClientOptions;
    client: mqtt.MqttClient;

    constructor(mqttJSON: any) {
        console.info(`MQTT : ${JSON.stringify(mqttJSON)}`);

        this.connect_opts = {
            host: mqttJSON.host,
            port: mqttJSON.port,
            protocol: mqttJSON.protocol,
            username: mqttJSON.user_name,
            password: mqttJSON.password
        };

        this.broker_url = `${mqttJSON.protocol}://${mqttJSON.host}:${mqttJSON.port}/mqtt`;
        this.client = mqtt.connect(this.broker_url, this.connect_opts);
        this.onConnect();
    };

    private onConnect(): void {
        this.client!.on("connect", () => {
            if (this.client!.connected) {
                console.log(`[MQTT] connected with [${this.broker_url}]`);
            }
            else {
                console.error('[MQTT] connection disconnected');
            }
        });
        this.client!.on("error", (err) => {
            console.error(`[MQTT] connection on ${err}`);
        });
    };

    public isConnected(): boolean {
        return this.client.connected;
    }

    public publish(topic: string, message: string): void {
        try {
            this.client!.publish(topic, message);
        } catch (error: any) {
            console.error(`[MQTT] publishing errror : ${error}`);
            throw new Error(error);
        };
    };

    public subscribe(topic: string): void {
        try {
            this.client!.subscribe(topic, function (err: Error | null, granted: mqtt.ISubscriptionGrant[] | undefined) {
                if (err) {
                    console.error(`[MQTT] ${topic} subscription on ${err}`);
                    return;
                };
                console.log(`[MQTT] subscription has granted by topic {${granted![0].topic}}`);
            });
        } catch (error: any) {
            console.error(`[MQTT] {${topic}} subscription : ${error}`);
            throw new Error(error);
        };
    };
};