//
// Created by antique on 24. 2. 22.
//

#include "model/Link.h"

void to_json(nlohmann::json &j, const Link &link) {
    j = nlohmann::json{
            {"linkId", link.linkId},
            {"stNode", link.stNode},
            {"edNode", link.edNode}
    };
}

void from_json(const nlohmann::json &j, Link &link) {
    j.at("linkId").get_to(link.linkId);
    j.at("stNode").get_to(link.stNode);
    j.at("edNode").get_to(link.edNode);
}