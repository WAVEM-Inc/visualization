//
// Created by antique on 24. 3. 5.
//

#include "enum/Direction.h"

std::string getDirectionName(Direction direction) {
    switch (direction) {
        case FORWARD:
            return "forward";
        case BACKWARD:
            return "backward";
        default:
            return "";
    }
}

std::string getDirectionKorName(Direction direction) {
    switch (direction) {
        case FORWARD:
            return "전방";
        case BACKWARD:
            return "후방";
        default:
            return "";
    }
}