//
// Created by antique on 24. 2. 20.
//

#include "struct/GraphNode.h"

void to_json(nlohmann::json& j, const GraphNode& node) {
    j = nlohmann::json{
            {"nodeId", node.nodeId},
            {"nodeName", node.nodeName},
            {"type", node.type},
            {"position", {{"latitude", node.position.latitude}, {"longitude", node.position.longitude}}}
    };
}

void from_json(const nlohmann::json& j, GraphNode& node) {
    j.at("nodeId").get_to(node.nodeId);
    j.at("nodeName").get_to(node.nodeName);
    j.at("type").get_to(node.type);
    j.at("position").at("latitude").get_to(node.position.latitude);
    j.at("position").at("longitude").get_to(node.position.longitude);
}