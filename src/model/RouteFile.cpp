//
// Created by antique on 24. 2. 22.
//

#include "model/RouteFile.h"

void to_json(nlohmann::json& j, const RouteFile& rf) {
    j = nlohmann::json{
            {"fileVersion", rf.fileVersion},
            {"mapId", rf.mapId},
            {"path", rf.path} // std::vector<Path>에 대한 to_json 함수는 자동으로 처리됩니다.
    };
}

void from_json(const nlohmann::json& j, RouteFile& rf) {
    j.at("fileVersion").get_to(rf.fileVersion);
    j.at("mapId").get_to(rf.mapId);
    j.at("path").get_to(rf.path); // std::vector<Path>에 대한 from_json 함수는 자동으로 처리됩니다.
}
