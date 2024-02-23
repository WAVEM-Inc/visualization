//
// Created by antique on 24. 2. 14.
//

#ifndef NODE_EDITOR_POSITION_H
#define NODE_EDITOR_POSITION_H

#include "nlohmann/json.hpp"

struct Position {
    double latitude;
    double longitude;

    Position() : latitude(0), longitude(0) {};
    Position(const double lat, const double lng) : latitude(lat), longitude(lng) {};
};

void to_json(nlohmann::json& j, const Position& p);

void from_json(const nlohmann::json& j, Position& p);


#endif //NODE_EDITOR_POSITION_H
