//
// Created by antique on 24. 3. 12.
//

#include "struct/ConfigFile.h"

void to_json(nlohmann::json& j, const ConfigFile& c) {
    j = nlohmann::json{
            {"center", c.center},
            {"zoomLevel", c.zoomLevel},
            {"latestFilePath", c.latestFilePath}
    };
}

void from_json(const nlohmann::json& j, ConfigFile& c) {
    j.at("center").get_to(c.center);
    j.at("zoomLevel").get_to(c.zoomLevel);
    j.at("latestFilePath").get_to(c.latestFilePath);
}