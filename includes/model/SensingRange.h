//
// Created by antique on 24. 2. 14.
//

#ifndef NODE_EDITOR_SENSINGRANGE_H
#define NODE_EDITOR_SENSINGRANGE_H

#include <string>
#include "Position.h"

struct SensingRange {
    Position position;
    double width;
    double height;
    std::string processingCode;

};

#endif //NODE_EDITOR_SENSINGRANGE_H
