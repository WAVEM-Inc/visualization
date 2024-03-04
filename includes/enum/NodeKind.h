//
// Created by antique on 24. 2. 29.
//

#ifndef NODE_EDITOR_NODEKIND_H
#define NODE_EDITOR_NODEKIND_H

#include <string>

enum NodeKine {
    INTERSECTION,
    CONNECTING,
    ENDPOINT,
    WAITING
};

std::string getKindName(NodeKine kind) {
    switch (kind) {
        case INTERSECTION:
            return "intersection";
        case CONNECTING:
            return "connecting";
        case ENDPOINT:
            return "endpoint";
        case WAITING:
            return "wating";
    }
}

std::string getKindKorName(NodeKine kind) {
    switch (kind) {
        case INTERSECTION:
            return "교차로";
        case CONNECTING:
            return "연결";
        case ENDPOINT:
            return "종점";
        case WAITING:
            return "일시정지";
    }
}

#endif //NODE_EDITOR_NODEKIND_H
