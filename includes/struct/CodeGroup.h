//
// Created by antique on 24. 3. 13.
//

#ifndef ROUTE_EDITOR_CODEGROUP_H
#define ROUTE_EDITOR_CODEGROUP_H

#include <string>
#include <vector>
#include "Code.h"

struct CodeGroup {
    std::string category;
    std::vector<Code> codes;

    CodeGroup() = default;
};

void to_json(nlohmann::json& j, const CodeGroup& codeGroup);

void from_json(const nlohmann::json& j, CodeGroup& codeGroup);

#endif //ROUTE_EDITOR_CODEGROUP_H
