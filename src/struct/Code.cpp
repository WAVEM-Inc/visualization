//
// Created by antique on 24. 3. 13.
//
#include "struct/Code.h"
#include "nlohmann/json.hpp"

void to_json(nlohmann::json& j, const Code& p) {
    j = nlohmann::json{{"code", p.code}, {"name", p.name}, {"description", p.description}};
}

// JSON 객체에서 Code 구조체로 역직렬화하는 함수
void from_json(const nlohmann::json& j, Code& p) {
    // JSON 객체에서 "code", "name", "description" 키를 찾아
    // 있으면 해당 값을 Code 구조체의 멤버 변수에 할당
    j.at("code").get_to(p.code);
    j.at("name").get_to(p.name);
    // "description" 키는 옵셔널이므로, 존재하지 않을 수 있음
    if (j.contains("description")) {
        j.at("description").get_to(p.description);
    } else {
        p.description = ""; // 기본값으로 빈 문자열 할당
    }
}