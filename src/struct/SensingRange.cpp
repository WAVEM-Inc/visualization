//
// Created by antique on 24. 2. 22.
//

#include "struct/SensingRange.h"

void to_json(nlohmann::json& j, const SensingRange& sr) {
    j = nlohmann::json{
            {"position", sr.position}, // Position 구조체에 대한 to_json 함수가 필요합니다.
            {"width", sr.width},
            {"height", sr.height},
            {"processingCode", sr.processingCode}
    };
}

void from_json(const nlohmann::json& j, SensingRange& sr) {
    j.at("position").get_to(sr.position); // Position 구조체에 대한 from_json 함수가 필요합니다.
    j.at("width").get_to(sr.width);
    j.at("height").get_to(sr.height);
    j.at("processingCode").get_to(sr.processingCode);
}