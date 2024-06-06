import * as fs from "fs";
import * as ini from "ini";
import * as os from "os";
import * as path from "path";
import * as configFilePathsJSON from "../../config/configFilePaths.json";

export default class PathService {

    constructor() {

    }

    public loadPath(): any | null {
        try {
            const pathFile: string = configFilePathsJSON.map_config_file_path;
            const homeDirectory: string = os.homedir();
            const mapConfigPath: string = path.join(homeDirectory, pathFile);
            console.info(`Map Config Path: ${mapConfigPath}`);

            let iniData: any;
            try {
                iniData = ini.parse(fs.readFileSync(mapConfigPath, 'utf-8'));
            } catch (error) {
                console.error(`Failed to read config file: ${error}`);
                return;
            }

            const mapFilePath: string = iniData.file_path;
            const mapFileName: string = iniData.file_name;

            const mapFileFullPath: string = path.join(homeDirectory, mapFilePath, mapFileName);
            console.info(`Map Path: ${mapFileFullPath}`);

            try {
                const fileContent = fs.readFileSync(mapFileFullPath, "utf-8");
                const pathData: any = JSON.parse(fileContent);

                const responseJSON: any = {
                    current_map_file: mapFileName,
                    paths: pathData
                }

                return responseJSON;
            } catch (error) {
                console.error(`Failed to read map file: ${error}`);
                return null;
            }
        } catch (error: any) {
            console.error(`Failed to read map file: ${error}`);
            return null;
        }
    }
}