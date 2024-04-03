//
// Created by antique on 24. 2. 22.
//

#include "struct/Position.h"

void to_json(nlohmann::json& j, const Position& p) {
    j = nlohmann::json{{"latitude", p.latitude}, {"longitude", p.longitude}};
}

void from_json(const nlohmann::json& j, Position& p) {
    j.at("latitude").get_to(p.latitude);
    j.at("longitude").get_to(p.longitude);
}