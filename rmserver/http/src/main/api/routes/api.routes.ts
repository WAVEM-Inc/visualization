import express from "express";
import APIController from "../presentation/api.controller";

const APIRouter: express.Router = express.Router();
const apiController: APIController = new APIController();

APIRouter.use((req, res, next) => {
    console.log("[rmserver_http] [INFO] APIRouter ready");
    next();
});

APIRouter.post(`/mqtt/load/config`, async (req, res) => {
    const mqttConfig: any | null = await apiController.loadMQTTConfig();

    if (mqttConfig !== null) {
        res.status(201).json(mqttConfig);
    } else {
        res.status(500).json({ error: "Failed to read mqtt.json file" });
    }
});

export default APIRouter;