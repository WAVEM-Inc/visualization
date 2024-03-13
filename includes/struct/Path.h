//
// Created by antique on 24. 2. 14.
//

#ifndef NODE_EDITOR_PATH_H
#define NODE_EDITOR_PATH_H

#include <iostream>
#include <utility>
#include "Node.h"
#include "vector"

struct Path {
    std::string id;
    std::string name;
    std::vector<Node> nodeList;

    Path() = default;
};

void to_json(nlohmann::json& j, const Path& p);

void from_json(const nlohmann::json& j, Path& p);

#endif //NODE_EDITOR_PATH_H
