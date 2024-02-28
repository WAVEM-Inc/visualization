//
// Created by antique on 24. 2. 27.
//

#ifndef NODE_EDITOR_NODECODE_H
#define NODE_EDITOR_NODECODE_H

#include <string>
#include "nlohmann/json.hpp"

struct NodeCode {
    int code;
    std::string name;
    std::string description;
};

void to_json(nlohmann::json& j, const NodeCode& nc);

void from_json(const nlohmann::json& j, NodeCode& nc);

#endif //NODE_EDITOR_NODECODE_H
