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

std::string getDirectionName(Direction direction) {
    switch (direction) {
        case FORWARD:
            return "forward";
        case BACKWARD:
            return "backward";
    }
}

std::string getDirectionKorName(Direction direction) {
    switch (direction) {
        case FORWARD:
            return "전방";
        case BACKWARD:
            return "후방";
    }
}

#endif //NODE_EDITOR_DIRECTION_H
