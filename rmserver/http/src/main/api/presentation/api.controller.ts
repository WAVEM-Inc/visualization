import APIService from "../application/api.service";

export default class APIController {

    private _apiService: APIService;

    constructor() {
        this._apiService = new APIService();
    }

    public async loadMQTTConfig(): Promise<any> {
        const mqttConfig: any = await this._apiService.loadMQTTConfig();
        return mqttConfig;
    }
}