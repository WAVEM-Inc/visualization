//
// Created by antique on 24. 2. 20.
//

#include "model/MapNode.h"

void to_json(nlohmann::json& j, const MapNode& mapNode) {
    j = nlohmann::json{
            {"nodeId", mapNode.nodeId},
            {"position", {{"latitude", mapNode.position.latitude}, {"longitude", mapNode.position.longitude}}}
    };
}

void from_json(const nlohmann::json& j, MapNode& mapNode) {
    j.at("nodeId").get_to(mapNode.nodeId);
    j.at("position").at("latitude").get_to(mapNode.position.latitude);
    j.at("position").at("longitude").get_to(mapNode.position.longitude);
}