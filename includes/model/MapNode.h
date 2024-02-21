//
// Created by antique on 24. 2. 14.
//

#ifndef NODE_EDITOR_MAPNODE_H
#define NODE_EDITOR_MAPNODE_H

#include <iostream>
#include "Position.h"
#include <fstream>
#include <utility>
#include "nlohmann/json.hpp"

struct MapNode {
    std::string nodeId;
    Position position;

    MapNode() : position(0, 0) {};
    MapNode(std::string id, const Position pos) : nodeId(std::move(id)), position(pos) {};
};

void to_json(nlohmann::json& j, const MapNode& mapNode);
void from_json(const nlohmann::json& j, MapNode& mapNode);

#endif //NODE_EDITOR_MAPNODE_H
