//
// Created by antique on 24. 2. 22.
//

#ifndef NODE_EDITOR_ROUTEFILE_H
#define NODE_EDITOR_ROUTEFILE_H

#include <iostream>
#include "nlohmann/json.hpp"
#include "Path.h"
#include "MapNode.h"

struct RouteFile {
    std::string version;
    std::string mapId;
    std::vector<Path> path;
    std::vector<MapNode> node;

    RouteFile() = default;
};

void to_json(nlohmann::json& j, const RouteFile& rf);

void from_json(const nlohmann::json& j, RouteFile& rf);

#endif //NODE_EDITOR_ROUTEFILE_H
