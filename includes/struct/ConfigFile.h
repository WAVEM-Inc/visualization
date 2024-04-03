//
// Created by antique on 24. 3. 12.
//

#ifndef ROUTE_EDITOR_CONFIGFILE_H
#define ROUTE_EDITOR_CONFIGFILE_H

#include "Position.h"

struct ConfigFile {
    Position center;
    int zoomLevel;
    std::string latestFilePath;

    ConfigFile() : center(36.112884, 128.369586), zoomLevel(18) {};
};

void to_json(nlohmann::json& j, const ConfigFile& c);

void from_json(const nlohmann::json& j, ConfigFile& c);

#endif //ROUTE_EDITOR_CONFIGFILE_H
