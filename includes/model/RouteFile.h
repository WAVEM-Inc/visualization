//
// Created by antique on 24. 2. 22.
//

#ifndef NODE_EDITOR_ROUTEFILE_H
#define NODE_EDITOR_ROUTEFILE_H

#include <iostream>
#include "nlohmann/json.hpp"
#include "Path.h"

struct RouteFile {
    std::string fileVersion;
    std::string mapId;
    std::vector<Path> path;

    RouteFile() = default;
};

void to_json(nlohmann::json& j, const RouteFile& rf);

void from_json(const nlohmann::json& j, RouteFile& rf);

#endif //NODE_EDITOR_ROUTEFILE_H
