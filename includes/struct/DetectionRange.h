//
// Created by antique on 24. 2. 14.
//

#ifndef NODE_EDITOR_DETECTIONRANGE_H
#define NODE_EDITOR_DETECTIONRANGE_H

#include <string>
#include "Position.h"

struct DetectionRange {
    Position position;
    double width;
    double height;
    std::string processingCode;

};

void to_json(nlohmann::json& j, const DetectionRange& sr);

void from_json(const nlohmann::json& j, DetectionRange& sr);

#endif //NODE_EDITOR_DETECTIONRANGE_H
