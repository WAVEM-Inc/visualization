import express from "express";
import PathController from "../presentation/path.controller";

const PathRouter: express.Router = express.Router();
const pathController: PathController = new PathController();

PathRouter.use((req, res, next) => {
    console.log("[rmserver_http] [INFO] PathRouter ready");
    next();
});

PathRouter.post(`/select`, async (req, res) => {
    const path: any | null = pathController.loadPath();

    if (path !== null) {
        res.status(201).json(path);
    } else {
        res.status(500).json({ error: "Failed to read PathMap file" });
    }
});

export default PathRouter;