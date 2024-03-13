//
// Created by antique on 24. 3. 13.
//

#include "struct/CodeGroup.h"

void to_json(nlohmann::json& j, const CodeGroup& codeGroup) {
    j = nlohmann::json{{"category", codeGroup.category}, {"codes", codeGroup.codes}};
}

void from_json(const nlohmann::json& j, CodeGroup& codeGroup) {
    j.at("category").get_to(codeGroup.category);
    j.at("codes").get_to(codeGroup.codes);
}