//
// Created by antique on 24. 2. 22.
//

#include "struct/Node.h"

void to_json(nlohmann::json& j, const Node& n) {
    j = nlohmann::json{
            {"nodeId", n.nodeId},
            {"position", n.position}, // Position 구조체에 대한 to_json 가정
            {"type", n.type},
            {"kind", n.kind},
            {"heading", n.heading},
            {"direction", n.direction},
            {"sensingRange", n.sensingRange} // SensingRange 구조체 배열에 대한 to_json 가정
    };
}

void from_json(const nlohmann::json& j, Node& n) {
    j.at("nodeId").get_to(n.nodeId);
    j.at("position").get_to(n.position); // Position 구조체에 대한 from_json 가정
    j.at("type").get_to(n.type);
    j.at("kind").get_to(n.kind);
    j.at("heading").get_to(n.heading);
    j.at("direction").get_to(n.direction);

    if (j.find("sensingRange") != j.end()) { // SensingRange가 존재하는 경우만 처리
        j.at("sensingRange").get_to(n.sensingRange); // SensingRange 구조체 배열에 대한 from_json 가정
    }
}