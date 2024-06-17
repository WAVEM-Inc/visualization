import cookieParser from "cookie-parser";
import cors from "cors";
import dotenv from "dotenv-flow";
import express, { NextFunction, Request, Response } from "express";
import http from "http";
import createHttpError from "http-errors";
import logger from "morgan";
import path from "path";
import { DEFAULT_API_FORMAT } from "./api/domain/api.constants";
import APIRouter from "./api/routes/api.routes";
import PathRouter from "./path/routes/path.routes";

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
app.use(`${DEFAULT_API_FORMAT}`, APIRouter);
app.use(`${DEFAULT_API_FORMAT}/path`, PathRouter);

const rmvizBuildPath: string = path.join(__dirname, "../../../../rmviz/build");
console.log(`[rmserver_http] [INFO] dirname : ${__dirname}, client_path : ${rmvizBuildPath}`);
app.use(express.static(rmvizBuildPath));

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

const server: http.Server = http.createServer(app);

app.listen(port, (): void => {
    console.log(`[rmserver_http] [INFO] Server is running on [${port}]`);
});

module.exports = app;
