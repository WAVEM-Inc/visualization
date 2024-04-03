//
// Created by antique on 24. 2. 14.
//

#ifndef NODE_EDITOR_LINK_H
#define NODE_EDITOR_LINK_H

#include <iostream>
#include "nlohmann/json.hpp"

struct Link {
    std::string linkId;
    std::string stNode;
    std::string edNode;
};

void to_json(nlohmann::json& j, const Link& link);

void from_json(const nlohmann::json& j, Link& mapNode);

#endif //NODE_EDITOR_LINK_H
