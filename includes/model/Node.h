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
    double heading;
    std::string direction;
    std::vector<SensingRange> sensingRange;
};

#endif //NODE_EDITOR_NODE_H
