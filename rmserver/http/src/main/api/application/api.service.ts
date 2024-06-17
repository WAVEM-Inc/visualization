import os from "os";
import path from "path";
import util from "util";
import fs from "fs";

export default class APIService {

    constructor() {

    }

    public async loadMQTTConfig(): Promise<any> {
        const homeDir: string = os.homedir();
        const mqttFilePath: string = path.join(homeDir, "RobotData/mqtt/mqtt.json");

        return new Promise<any>((resolve, reject) => {
            try {
                util.promisify(fs.readFile)(mqttFilePath, "utf8")
                    .then((value: string) => {
                        console.info(`APIService : ${value}`);
                        resolve(JSON.parse(value));
                    })
                    .catch((error: any) => {
                        reject(error);
                    });
            } catch (error: any) {
                reject(error);
            }
        });
    }
}