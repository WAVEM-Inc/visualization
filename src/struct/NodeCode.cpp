//
// Created by antique on 24. 2. 27.
//

#include "struct/NodeCode.h"

void to_json(nlohmann::json& j, const NodeCode& nc) {
    j = nlohmann::json{
            {"code", nc.code},
            {"name", nc.name},
            {"description", nc.description}
    };
}

void from_json(const nlohmann::json& j, NodeCode& nc) {
    j.at("code").get_to(nc.code);
    j.at("name").get_to(nc.name);
    j.at("description").get_to(nc.description);
}