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

struct GraphNode {
    std::string nodeId;
    std::string nodeName;
    std::string type;
    Position position;

    GraphNode() = default;
    GraphNode(std::string id, const Position pos) : nodeId(std::move(id)), position(pos), nodeName(std::move(id)) {};
};

void to_json(nlohmann::json& j, const GraphNode& node);

void from_json(const nlohmann::json& j, GraphNode& node);

#endif //NODE_EDITOR_MAPNODE_H
