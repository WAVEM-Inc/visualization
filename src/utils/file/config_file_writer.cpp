//
// Created by antique on 24. 3. 12.
//

#include <string>
#include <fstream>
#include <iostream>
#include "utils/file/config_file_writer.h"
#include "model/map_state_model.h"
#include "struct/ConfigFile.h"
#include "model/file_info_model.h"

ConfigFileWriter::ConfigFileWriter() = default;

ConfigFileWriter::~ConfigFileWriter() = default;

bool ConfigFileWriter::saveFile() {
    std::string filePath(RESOURCE_DIR);
    filePath += "/config/config.json";

    ConfigFile data = ConfigFile();
    data.center = MapStateModel::getInstance().getCenterLocation();
    data.zoomLevel = MapStateModel::getInstance().getZoomLevel();
    data.latestFilePath = FileInfoModel::getInstance().getLatestFilePath();

    nlohmann::json jsonData = data;

    std::ofstream outFile(filePath);
    if (!outFile) {
        std::cout << "Error: Can not write file to " << filePath << ".\n";
        return false;
    }

    outFile << jsonData.dump(4) << std::endl;

    outFile.close();

    return true;
}
