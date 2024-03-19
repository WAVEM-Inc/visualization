//
// Created by antique on 24. 2. 14.
//

#ifndef NODE_EDITOR_NODE_H
#define NODE_EDITOR_NODE_H

#include <iostream>
#include <vector>
#include "Position.h"
#include "DetectionRange.h"

#include <fstream>
#include <nlohmann/json.hpp>

struct Node {
    std::string nodeId;
    Position position;
    std::string type;
    std::string kind;
    int heading = -1;
    std::string direction;
    std::vector<DetectionRange> detectionRange;

    Node() = default;
};

void to_json(nlohmann::json& j, const Node& n);

void from_json(const nlohmann::json& j, Node& n);

#endif //NODE_EDITOR_NODE_H
