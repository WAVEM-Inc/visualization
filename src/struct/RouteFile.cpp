//
// Created by antique on 24. 2. 22.
//

#include "struct/RouteFile.h"

void to_json(nlohmann::json& j, const RouteFile& rf) {
    j = nlohmann::json{
            {"version", rf.version},
            {"mapId", rf.mapId},
            {"path", rf.path},
            {"node", rf.node}
    };
}

void from_json(const nlohmann::json& j, RouteFile& rf) {
    j.at("version").get_to(rf.version);
    j.at("mapId").get_to(rf.mapId);
    j.at("path").get_to(rf.path);
    j.at("node").get_to(rf.node);
}
