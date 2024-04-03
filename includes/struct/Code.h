//
// Created by antique on 24. 3. 13.
//

#ifndef ROUTE_EDITOR_CODE_H
#define ROUTE_EDITOR_CODE_H

#include <string>
#include <utility>
#include "nlohmann/json.hpp"

struct Code {
    std::string code;
    std::string name;
    std::string description;

    Code() = default;
    Code(std::string code, std::string name) : code(std::move(code)), name(std::move(name)) {};
    Code(std::string code, std::string name, std::string description) : code(std::move(code)), name(std::move(name)), description(std::move(description)) {};
};

void to_json(nlohmann::json& j, const Code& p);

void from_json(const nlohmann::json& j, Code& p);

#endif //ROUTE_EDITOR_CODE_H
