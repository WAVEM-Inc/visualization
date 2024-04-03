//
// Created by antique on 24. 2. 22.
//

#include "struct/DetectionRange.h"

void to_json(nlohmann::json& j, const DetectionRange& sr) {
    j = nlohmann::json{
            {"offset", sr.offset}, // Position 구조체에 대한 to_json 함수가 필요합니다.
            {"widthLeft", sr.widthLeft},
            {"widthRight", sr.widthRight},
            {"height", sr.height},
            {"actionCode", sr.actionCode}
    };
}

void from_json(const nlohmann::json& j, DetectionRange& sr) {
    j.at("offset").get_to(sr.offset); // Position 구조체에 대한 from_json 함수가 필요합니다.
    j.at("widthLeft").get_to(sr.widthLeft);
    j.at("widthRight").get_to(sr.widthRight);
    j.at("height").get_to(sr.height);
    j.at("actionCode").get_to(sr.actionCode);
}