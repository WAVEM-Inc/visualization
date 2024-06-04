import cookieParser from "cookie-parser";
import cors from "cors";
import dotenv from "dotenv-flow";
import express, { NextFunction, Request, Response } from "express";
import fs from "fs";
import http from "http";
import createHttpError from "http-errors";
import logger from "morgan";
import os from "os";
import path from "path";
import util from "util";
dotenv.config();

const app: express.Application = express();
const port: number = parseInt(process.env.SERVER_BASE_URL || "3000", 10);

if (isNaN(port) || port < 0 || port >= 65536) {
    throw new RangeError(`Port number is not valid: ${port}`);
}

app.use(logger("dev"));
app.use(express.json());
app.use(express.urlencoded({ extended: false }));
app.use(cookieParser());
app.use(cors());

const rmvizBuildPath: string = path.join(__dirname, "../../../rmviz/build");
console.log(`[rmserver_http] [INFO] dirname : ${__dirname}, client_path : ${rmvizBuildPath}`);
app.use(express.static(rmvizBuildPath));

const homeDir: string = os.homedir();
const mqttFilePath: string = path.join(homeDir, "RobotData/mqtt/mqtt.json");

app.get("/v1/api/mqtt/load/config", async (req: Request, res: Response) => {
    try {
        const data: string = await util.promisify(fs.readFile)(mqttFilePath, "utf8");
        console.info(`mqttData : ${data}`);
        res.json(JSON.parse(data));
    } catch (err) {
        res.status(500).json({ error: "Failed to read mqtt.json file" });
    }
});

app.get("*", (req: Request, res: Response) => {
    res.sendFile(path.join(rmvizBuildPath, "index.html"));
});

app.use(function (req: Request, res: Response, next: NextFunction) {
    next(createHttpError(404));
});

app.use(function (err: any, req: Request, res: Response, next: NextFunction) {
    res.status(err.status || 500);
    res.json({
        message: err.message,
        error: req.app.get("env") === "development" ? err : {}
    });
});

const server = http.createServer(app);

app.listen(port, () => {
    console.log(`[rmserver_http] [INFO] Server is running at port <${port}>`);
});

module.exports = app;
