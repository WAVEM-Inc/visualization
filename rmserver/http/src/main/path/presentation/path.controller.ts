import PathService from "../application/path.service";

export default class PathController {

    private _pathService: PathService;

    constructor() {
        this._pathService = new PathService();
    }

    public loadPath(): any | null {
        return this._pathService.loadPath();
    }
}