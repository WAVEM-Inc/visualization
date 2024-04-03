//
// Created by antique on 24. 3. 12.
//

#include <fstream>
#include "utils/file/config_file_reader.h"
#include "utils/file/config_file_writer.h"
#include "model/map_state_model.h"

ConfigFileReader::ConfigFileReader() = default;

ConfigFileReader::~ConfigFileReader() = default;

ConfigFile ConfigFileReader::loadFile() {
    std::string filePath(RESOURCE_DIR);
    filePath += "/config/config.json";
    std::ifstream file(filePath);
    std::string fileContents;
    ConfigFile cfgData;

    if (file) {
        std::stringstream buffer;
        buffer << file.rdbuf();
        fileContents = buffer.str();
    } else {
        ConfigFileWriter writer;
        writer.saveFile();
        cfgData = loadFile();
    }

    nlohmann::json json = nlohmann::json::parse(fileContents);
    cfgData = json.get<ConfigFile>();

    file.close();

    return cfgData;
}
