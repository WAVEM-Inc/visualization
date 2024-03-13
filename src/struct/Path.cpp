//
// Created by antique on 24. 2. 22.
//

#include "struct/Path.h"

void to_json(nlohmann::json& j, const Path& p) {
    j = nlohmann::json{
            {"id", p.id},
            {"name", p.name},
            {"nodeList", p.nodeList} // Node 구조체 배열에 대한 to_json 함수가 이미 정의되어 있어야 합니다.
    };
}

void from_json(const nlohmann::json& j, Path& p) {
    j.at("id").get_to(p.id);
    j.at("name").get_to(p.name);
    j.at("nodeList").get_to(p.nodeList); // Node 구조체 배열에 대한 from_json 함수가 이미 정의되어 있어야 합니다.
}