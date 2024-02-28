//
// Created by antique on 24. 2. 14.
//

#ifndef NODE_EDITOR_NODE_H
#define NODE_EDITOR_NODE_H

#include <iostream>
#include <vector>
#include "Position.h"
#include "SensingRange.h"

#include <fstream>
#include <nlohmann/json.hpp>

struct Node {
    std::string nodeId;
    Position position;
    std::string type;
    std::string kind;
    int heading;
    std::string direction;
    std::vector<SensingRange> sensingRange;

    Node() = default;
};

void to_json(nlohmann::json& j, const Node& n);

void from_json(const nlohmann::json& j, Node& n);

#endif //NODE_EDITOR_NODE_H
