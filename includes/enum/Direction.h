//
// Created by antique on 24. 2. 29.
//

#ifndef NODE_EDITOR_DIRECTION_H
#define NODE_EDITOR_DIRECTION_H

#include <string>

enum Direction {
    FORWARD,
    BACKWARD
};

std::string getDirectionName(Direction direction);

std::string getDirectionKorName(Direction direction);

#endif //NODE_EDITOR_DIRECTION_H
